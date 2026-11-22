// Modified 2026 by foxyv: USB transfer error handling (no endless resubmit,
// no close_transfers() from the callback, correct active-transfer count).
#include "urb.hpp"
#include "mgr.hpp"

#include <algorithm>
#include <optional>
#include <libusb.h>

namespace ps3eye::detail {

/* Values for bmHeaderInfo = (Video and Still Image Payload Headers, 2.4.3.3) */
enum {
    UVC_STREAM_EOH = 1 << 7,
    UVC_STREAM_ERR = 1 << 6,
    UVC_STREAM_STI = 1 << 5,
    UVC_STREAM_RES = 1 << 4,
    UVC_STREAM_SCR = 1 << 3,
    UVC_STREAM_PTS = 1 << 2,
    UVC_STREAM_EOF = 1 << 1,
    UVC_STREAM_FID = 1 << 0,
};

urb_descriptor::urb_descriptor() = default;

static void LIBUSB_CALL transfer_completed_callback(struct libusb_transfer* xfr)
{
    urb_descriptor* urb = reinterpret_cast<urb_descriptor*>(xfr->user_data);
    enum libusb_transfer_status status = xfr->status;

    switch (status)
    {
    case LIBUSB_TRANSFER_COMPLETED:
        urb->consecutive_errors = 0;
        urb->pkt_scan(xfr->buffer, xfr->actual_length);
        break;
    case LIBUSB_TRANSFER_CANCELLED:
        // close_transfers() asked for this; never resubmit.
        return urb->transfer_retired();
    case LIBUSB_TRANSFER_NO_DEVICE:
    case LIBUSB_TRANSFER_STALL:
        // Device is gone or the endpoint is halted. Resubmitting would
        // fail (or STALL again) in a tight loop and can push the xHCI
        // controller into resetting the port. Retire this transfer; the
        // halt is cleared in start_transfers() on the next start.
        ps3eye_debug("transfer error %d, retiring transfer", status);
        urb->frame_add(DISCARD_PACKET, nullptr, 0);
        return urb->transfer_failed(status);
    default: // ERROR, TIMED_OUT, OVERFLOW
        ps3eye_debug("transfer error %d", status);
        urb->frame_add(DISCARD_PACKET, nullptr, 0);
        if (++urb->consecutive_errors >= urb_descriptor::max_consecutive_errors)
        {
            ps3eye_debug("too many consecutive transfer errors, retiring transfer");
            return urb->transfer_failed(status);
        }
        break;
    }

    if (urb->stopping)
        return urb->transfer_retired();

    if (libusb_submit_transfer(xfr) < 0)
    {
        // NOTE: never call close_transfers() here. This runs on the libusb
        // event thread, and close_transfers() waits for cancellations that
        // only this thread can deliver -> deadlock.
        ps3eye_debug("error re-submitting URB");
        urb->transfer_failed(-1);
    }
}

/*
 * look for an input transfer endpoint in an alternate setting
 * libusb_endpoint_descriptor
 */
static uint8_t find_ep(struct libusb_device* device)
{
    const struct libusb_interface_descriptor* altsetting = nullptr;
    const struct libusb_endpoint_descriptor* ep;
    struct libusb_config_descriptor* config = nullptr;
    int i;
    uint8_t ep_addr = 0;

    libusb_get_active_config_descriptor(device, &config);

    if (!config) return 0;

    for (i = 0; i < config->bNumInterfaces; i++)
    {
        altsetting = config->interface[i].altsetting;
        if (altsetting[0].bInterfaceNumber == 0)
        {
            break;
        }
    }

    if (!altsetting)
        return 0;

    for (i = 0; i < altsetting->bNumEndpoints; i++)
    {
        ep = &altsetting->endpoint[i];
        if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK &&
            ep->wMaxPacketSize != 0)
        {
            ep_addr = ep->bEndpointAddress;
            break;
        }
    }

    libusb_free_config_descriptor(config);

    return ep_addr;
}

bool urb_descriptor::start_transfers(libusb_device_handle* handle, uint32_t frame_size_)
{
    // Initialize the frame queue
    frame_size = frame_size_;
    queue.init(frame_size);

    // Initialize the current frame pointer to the start of the buffer; it
    // will be updated as frames are completed and pushed onto the frame queue
    cur_frame_start = queue.buffer();
    frame_data_len = 0;

    // Find the bulk transfer endpoint
    uint8_t bulk_endpoint = find_ep(libusb_get_device(handle));
    libusb_clear_halt(handle, bulk_endpoint);

    // Allocate the transfer buffer
    memset(transfer_buffer.data(), 0, transfer_size * num_transfers);

    consecutive_errors = 0;
    stopping = false;
    last_pts = 0;
    last_fid = 0;
    last_packet_type = DISCARD_PACKET;

    bool ok = true;
    {
        std::lock_guard<std::mutex> lock(num_active_transfers_mutex);
        for (unsigned i = 0; i < num_transfers; ++i)
        {
            // Create & submit the transfer
            xfr[i] = libusb_alloc_transfer(0);
            if (!xfr[i]) { ok = false; break; }
            libusb_fill_bulk_transfer(xfr[i], handle, bulk_endpoint,
                                      transfer_buffer.data() + i * transfer_size, transfer_size, transfer_completed_callback,
                                      reinterpret_cast<void*>(this), 0);

            // Only count transfers that were actually submitted, otherwise
            // close_transfers() waits forever for callbacks that never come.
            if (libusb_submit_transfer(xfr[i]) == 0)
                num_active_transfers++;
            else
                ok = false;
        }
    }

    streaming = true;
    usb_manager::instance().camera_started();

    return ok;
}

void urb_descriptor::close_transfers()
{
    std::unique_lock<std::mutex> lock(num_active_transfers_mutex);
    if (!streaming) return;

    stopping = true;

    // Cancel any pending transfers (already-retired ones just return NOT_FOUND)
    for (unsigned i = 0; i < num_transfers; ++i)
        if (xfr[i])
            libusb_cancel_transfer(xfr[i]);

    // Wait for every in-flight transfer to report back. The libusb event
    // thread is still running at this point, so this can't deadlock.
    num_active_transfers_condition.wait(lock, [this]() {
        return num_active_transfers == 0;
    });

    // Free all transfers
    for (unsigned i = 0; i < num_transfers; ++i)
    {
        if (!xfr[i])
            continue;

        libusb_free_transfer(xfr[i]);
        xfr[i] = nullptr;
    }

    streaming = false;
    stopping = false;
    lock.unlock();
    usb_manager::instance().camera_stopped();
}

void urb_descriptor::transfer_retired()
{
    std::lock_guard<std::mutex> lock(num_active_transfers_mutex);
    if (num_active_transfers > 0)
        --num_active_transfers;
    num_active_transfers_condition.notify_one();
}

void urb_descriptor::transfer_failed(int status)
{
    (void)status;
    // The transfer is no longer in flight. The camera stops producing frames,
    // and the caller's next stop/start cycle clears the halt and resubmits.
    transfer_retired();
}

void urb_descriptor::frame_add(enum gspca_packet_type packet_type, const uint8_t* data, int len)
{
    if (packet_type == FIRST_PACKET)
    {
        frame_data_len = 0;
    }
    else
    {
        switch (last_packet_type) // ignore warning.
        {
        case DISCARD_PACKET:
            if (packet_type == LAST_PACKET)
            {
                last_packet_type = packet_type;
                frame_data_len = 0;
            }
            return;
        case LAST_PACKET:
            return;
        default:
            break;
        }
    }

    /* append the packet to the frame buffer */
    if (len > 0)
    {
        if (frame_data_len + (unsigned)len > frame_size)
        {
            packet_type = DISCARD_PACKET;
            frame_data_len = 0;
        }
        else
        {
            memcpy(cur_frame_start + frame_data_len, data, (unsigned)len);
            frame_data_len += (unsigned)len;
        }
    }

    last_packet_type = packet_type;

    if (packet_type == LAST_PACKET)
    {
        frame_data_len = 0;
        cur_frame_start = queue.enqueue();
        // debug("frame completed %d\n", frame_complete_ind);
    }
}

void urb_descriptor::pkt_scan(uint8_t* data, int len)
{
    uint32_t this_pts;
    uint16_t this_fid;
    int remaining_len = len;
    int payload_len;

    payload_len = 2048; // bulk type
    do
    {
        len = std::min(remaining_len, payload_len);

        /* Payloads are prefixed with a UVC-style header.  We
           consider a frame to start when the FID toggles, or the PTS
           changes.  A frame ends when EOF is set, and we've received
           the correct number of bytes. */

        /* Verify UVC header.  Header length is always 12 */
        if (data[0] != 12 || len < 12)
        {
            ps3eye_debug("bad header");
            goto discard;
        }

        /* Check errors */
        if (data[1] & UVC_STREAM_ERR)
        {
            ps3eye_debug("payload error, data[1]=%u", data[1]);
            goto discard;
        }

        /* Extract PTS and FID */
        if (!(data[1] & UVC_STREAM_PTS))
        {
            ps3eye_debug("PTS not present");
            goto discard;
        }

        this_pts = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
        this_fid = !!(data[1] & UVC_STREAM_FID);

        /* If PTS or FID has changed, start a new frame. */
        if (this_pts != last_pts || this_fid != last_fid)
        {
            if (last_packet_type == INTER_PACKET)
            {
                /* The last frame was incomplete, so don't keep it or we
                 * will glitch */
                frame_add(DISCARD_PACKET, nullptr, 0);
            }
            last_pts = this_pts;
            last_fid = this_fid;
            frame_add(FIRST_PACKET, data + 12, len - 12);
        } /* If this packet is marked as EOF, end the frame */
        else if (data[1] & UVC_STREAM_EOF)
        {
            last_pts = 0;
            if (frame_data_len + (unsigned)len - 12 != frame_size)
            {
                goto discard;
            }
            frame_add(LAST_PACKET, data + 12, len - 12);
        }
        else
        {
            /* Add the data from this payload */
            frame_add(INTER_PACKET, data + 12, len - 12);
        }

        /* Done this payload */
        goto scan_next;

discard:
        /* Discard data until a new frame starts. */
        frame_add(DISCARD_PACKET, nullptr, 0);
scan_next:
        remaining_len -= len;
        data += len;
    } while (remaining_len > 0);
}

urb_descriptor::~urb_descriptor()
{
    //ps3eye_debug("urb_descriptor destructor");
    close_transfers();
}

} // namespace ps3eye
