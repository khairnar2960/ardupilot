/*
   Simulator mixin for MAVLink Camera Protocol v2 peripherals.
*/

#include "SIM_config.h"

#if AP_SIM_MAVLINKCAMV2_ENABLED

#include "SIM_MAVLinkCamV2.h"
#include "SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Location.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace SITL {

void MAVLinkCamV2::set_camera_instance(uint8_t instance)
{
    _camera_compid = MIN(MAV_COMP_ID_CAMERA + instance, (uint8_t)MAV_COMP_ID_CAMERA6);
}

void MAVLinkCamV2::set_zoom_bridge(const char *address, uint16_t port)
{
    _zoom_bridge_address = address;
    _zoom_bridge_port = port;
    if (!_zoom_bridge_bound) {
        _zoom_bridge_bound = _zoom_bridge_socket.bind("127.0.0.1", 9011);
        if (!_zoom_bridge_bound) {
            ::printf("MAVLinkCamV2[compid=%u]: failed to bind Argos result port 9011\n",
                     (unsigned)_camera_compid);
        }
    }
}

void MAVLinkCamV2::send_zoom_to_bridge(float percent)
{
    if (_zoom_bridge_address == nullptr || _zoom_bridge_port == 0) {
        return;
    }

    char packet[32];
    const int length = snprintf(packet, sizeof(packet), "%.9g", (double)percent);
    if (length <= 0 || (size_t)length >= sizeof(packet) ||
        _zoom_bridge_socket.sendto(packet, (size_t)length,
                                   _zoom_bridge_address,
                                   _zoom_bridge_port) != length) {
        ::printf("MAVLinkCamV2[compid=%u]: failed to send zoom to Gazebo bridge\n",
                 (unsigned)_camera_compid);
    }
}

bool MAVLinkCamV2::send_capture_to_bridge(uint32_t request_id,
                                         int32_t image_index)
{
    if (!_zoom_bridge_bound || _zoom_bridge_address == nullptr ||
        _zoom_bridge_port == 0 || !_aircraft_state_valid) {
        return false;
    }

    const uint32_t time_boot_ms = AP_HAL::millis();

    _pending_bridge_command = {};
    _pending_bridge_command.active = true;
    _pending_bridge_command.request_id = request_id;
    _pending_bridge_command.command = MAV_CMD_IMAGE_START_CAPTURE;
    _pending_bridge_command.time_boot_ms = time_boot_ms;
    _pending_bridge_command.image_index = image_index;
    _pending_bridge_command.lat = _aircraft_lat;
    _pending_bridge_command.lon = _aircraft_lon;
    _pending_bridge_command.alt_mm = _aircraft_alt_mm;
    _pending_bridge_command.relative_alt_mm = _aircraft_relative_alt_mm;
    memcpy(_pending_bridge_command.vehicle_q, _aircraft_q,
           sizeof(_aircraft_q));
    _pending_bridge_command.started_ms = AP_HAL::millis();

    char packet[256];
    const int length = snprintf(
        packet, sizeof(packet),
        "C|%u|%d|%u|%d|%d|%d|%d|%.9g|%.9g|%.9g|%.9g",
        (unsigned)request_id, (int)image_index, (unsigned)time_boot_ms,
        (int)_aircraft_lat, (int)_aircraft_lon,
        (int)_pending_bridge_command.alt_mm,
        (int)_pending_bridge_command.relative_alt_mm,
        (double)_aircraft_q[0], (double)_aircraft_q[1],
        (double)_aircraft_q[2], (double)_aircraft_q[3]);
    if (length <= 0 || (size_t)length >= sizeof(packet) ||
        _zoom_bridge_socket.sendto(packet, (size_t)length,
                                   _zoom_bridge_address,
                                   _zoom_bridge_port) != length) {
        _pending_bridge_command.active = false;
        return false;
    }
    return true;
}

bool MAVLinkCamV2::send_video_to_bridge(uint32_t request_id, bool start)
{
    if (!_zoom_bridge_bound || _zoom_bridge_address == nullptr ||
        _zoom_bridge_port == 0) {
        return false;
    }
    char packet[48];
    const int length = snprintf(packet, sizeof(packet), "V|%u|%u",
                                (unsigned)request_id, start ? 1U : 0U);
    return length > 0 && (size_t)length < sizeof(packet) &&
        _zoom_bridge_socket.sendto(packet, (size_t)length,
                                   _zoom_bridge_address,
                                   _zoom_bridge_port) == length;
}

void MAVLinkCamV2::complete_capture(const char *response, uint32_t request_id)
{
    unsigned response_id = 0;
    char status[8] {};
    if (sscanf(response, "C|%u|%7[^|]", &response_id, status) != 2 ||
        response_id != request_id) {
        ::printf("MAVLinkCamV2[compid=%u]: invalid Argos capture response\n",
                 (unsigned)_camera_compid);
        return;
    }
    if (strcmp(status, "OK") != 0) {
        send_camera_command_ack(_pending_bridge_command.target_sysid,
                                _pending_bridge_command.target_compid,
                                MAV_CMD_IMAGE_START_CAPTURE,
                                MAV_RESULT_FAILED);
        _pending_bridge_command.active = false;
        return;
    }

    char file_url[205] {};
    unsigned long long time_utc = 0;
    float q[4] {};
    if (sscanf(response, "C|%u|%7[^|]|%llu|%204[^|]|%f|%f|%f|%f",
               &response_id, status, &time_utc, file_url,
               &q[0], &q[1], &q[2], &q[3]) != 8 ||
        response_id != request_id ||
        !isfinite(q[0]) || !isfinite(q[1]) ||
        !isfinite(q[2]) || !isfinite(q[3])) {
        ::printf("MAVLinkCamV2[compid=%u]: invalid Argos capture response\n",
                 (unsigned)_camera_compid);
        return;
    }

    _camera.trigger_shutter();
    mavlink_camera_image_captured_t captured {};
    captured.time_boot_ms = AP_HAL::millis();
    captured.time_utc = time_utc;
    captured.camera_id = 0;
    captured.lat = _pending_bridge_command.lat;
    captured.lon = _pending_bridge_command.lon;
    captured.alt = _pending_bridge_command.alt_mm;
    captured.relative_alt = _pending_bridge_command.relative_alt_mm;
    for (uint8_t i = 0; i < 4; i++) {
        captured.q[i] = q[i];
    }
    captured.image_index = _pending_bridge_command.image_index;
    captured.capture_result = 1;
    strncpy_noterm(captured.file_url, file_url, sizeof(captured.file_url));

    mavlink_message_t msg;
    mavlink_msg_camera_image_captured_encode_status(
        camera_vehicle_sysid(), _camera_compid, &camera_mav_status(),
        &msg, &captured);
    camera_send_mavlink_message(msg);
    send_camera_command_ack(_pending_bridge_command.target_sysid,
                            _pending_bridge_command.target_compid,
                            MAV_CMD_IMAGE_START_CAPTURE,
                            MAV_RESULT_ACCEPTED);
    send_camera_capture_status();
    _pending_bridge_command.active = false;
}

void MAVLinkCamV2::receive_bridge_results()
{
    if (!_zoom_bridge_bound) {
        return;
    }
    char response[1024];
    while (true) {
        const ssize_t received =
            _zoom_bridge_socket.recv(response, sizeof(response) - 1, 0);
        if (received <= 0) {
            return;
        }
        response[received] = '\0';

        unsigned request_id = 0;
        char kind = '\0';
        char status[8] {};
        if (sscanf(response, "%c|%u|%7[^|]", &kind, &request_id, status) != 3 ||
            !_pending_bridge_command.active ||
            request_id != _pending_bridge_command.request_id) {
            ::printf("MAVLinkCamV2[compid=%u]: ignoring unmatched Argos bridge response\n",
                     (unsigned)_camera_compid);
            continue;
        }

        if (_pending_bridge_command.command == MAV_CMD_IMAGE_START_CAPTURE &&
            kind == 'C') {
            complete_capture(response, request_id);
            continue;
        }
        if (kind != 'V' ||
            (_pending_bridge_command.command != MAV_CMD_VIDEO_START_CAPTURE &&
             _pending_bridge_command.command != MAV_CMD_VIDEO_STOP_CAPTURE)) {
            ::printf("MAVLinkCamV2[compid=%u]: unexpected Argos bridge response\n",
                     (unsigned)_camera_compid);
            continue;
        }

        const MAV_CMD command = _pending_bridge_command.command;
        const uint8_t target_sysid = _pending_bridge_command.target_sysid;
        const uint8_t target_compid = _pending_bridge_command.target_compid;
        const bool succeeded = strcmp(status, "OK") == 0;
        if (succeeded) {
            const bool recording = command == MAV_CMD_VIDEO_START_CAPTURE;
            _camera.set_recording(recording);
            _recording_started_ms = recording ? AP_HAL::millis() : 0;
        }
        send_camera_command_ack(target_sysid, target_compid, command,
                                succeeded ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED);
        send_camera_capture_status();
        _pending_bridge_command.active = false;
    }
}

void MAVLinkCamV2::update(const class Aircraft &aircraft)
{
    if (camera_vehicle_sysid() == 0) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    const Location &location = aircraft.get_location();
    int32_t alt_cm = 0;
    _aircraft_state_valid =
        location.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm);
    _aircraft_lat = location.lat;
    _aircraft_lon = location.lng;
    _aircraft_alt_mm = _aircraft_state_valid ? alt_cm * 10 : 0;
    _aircraft_relative_alt_mm =
        static_cast<int32_t>(lrint(
            aircraft.get_height_above_ground() * 1000.0f));
    Quaternion attitude;
    aircraft.get_attitude(attitude);
    _aircraft_q[0] = attitude.q1;
    _aircraft_q[1] = attitude.q2;
    _aircraft_q[2] = attitude.q3;
    _aircraft_q[3] = attitude.q4;
    receive_bridge_results();
    if (_pending_bridge_command.active &&
        now_ms - _pending_bridge_command.started_ms > 10000) {
        send_camera_command_ack(_pending_bridge_command.target_sysid,
                                _pending_bridge_command.target_compid,
                                _pending_bridge_command.command,
                                MAV_RESULT_FAILED);
        _pending_bridge_command.active = false;
    }
    if (now_ms - _last_camera_heartbeat_ms >= 1000) {
        _last_camera_heartbeat_ms = now_ms;
        send_camera_heartbeat();
    }
}

void MAVLinkCamV2::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
        return;
    }
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    uint32_t target_system;
    mavlink_msg_get_target_system(&msg, &cmd.target_system, &target_system);
    if (target_system != camera_vehicle_sysid() ||
        cmd.target_component != _camera_compid) {
        return;
    }
    switch ((MAV_CMD)cmd.command) {
    case MAV_CMD_REQUEST_MESSAGE:
        if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
            send_camera_information(msg.sysid, msg.compid);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
        } else if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_CAMERA_SETTINGS) {
            send_camera_settings();
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
        } else if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
            send_camera_capture_status();
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
        } else if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION) {
            MAV_RESULT result = MAV_RESULT_DENIED;
            const uint8_t stream_count = get_video_stream_count();
            if (isfinite(cmd.param2) && cmd.param2 >= 0 &&
                cmd.param2 <= stream_count) {
                const uint8_t requested_stream = (uint8_t)cmd.param2;
                if (is_equal(cmd.param2, (float)requested_stream)) {
                    if (requested_stream == 0) {
                        result = stream_count > 0 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
                        for (uint8_t stream_id = 1; stream_id <= stream_count; stream_id++) {
                            if (!send_video_stream_information(stream_id)) {
                                result = MAV_RESULT_FAILED;
                            }
                        }
                    } else if (send_video_stream_information(requested_stream)) {
                        result = MAV_RESULT_ACCEPTED;
                    }
                }
            }
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, result);
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED);
        }
        break;
    case MAV_CMD_IMAGE_START_CAPTURE:
        if (_pending_bridge_command.active) {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_IMAGE_START_CAPTURE,
                                    MAV_RESULT_TEMPORARILY_REJECTED);
            break;
        }
        if (!isfinite(cmd.param3) || !is_equal(cmd.param3, 1.0f)) {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_IMAGE_START_CAPTURE,
                                    MAV_RESULT_DENIED);
            break;
        }
        {
            const uint32_t request_id = _next_bridge_request_id++;
            const int32_t image_index = isfinite(cmd.param4) && cmd.param4 >= 1.0f
                ? static_cast<int32_t>(cmd.param4) - 1
                : static_cast<int32_t>(_camera.shot_count());
            _pending_bridge_command.target_sysid = msg.sysid;
            _pending_bridge_command.target_compid = msg.compid;
            if (!send_capture_to_bridge(request_id, image_index)) {
                _pending_bridge_command.active = false;
                send_camera_command_ack(msg.sysid, msg.compid,
                                        MAV_CMD_IMAGE_START_CAPTURE,
                                        MAV_RESULT_FAILED);
            } else {
                _pending_bridge_command.target_sysid = msg.sysid;
                _pending_bridge_command.target_compid = msg.compid;
            }
        }
        break;
    case MAV_CMD_IMAGE_STOP_CAPTURE:
        send_camera_command_ack(msg.sysid, msg.compid,
                                MAV_CMD_IMAGE_STOP_CAPTURE, MAV_RESULT_ACCEPTED);
        send_camera_capture_status();
        break;
    case MAV_CMD_VIDEO_START_CAPTURE:
        if (_camera.recording()) {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_VIDEO_START_CAPTURE,
                                    MAV_RESULT_ACCEPTED);
            send_camera_capture_status();
        } else if (_pending_bridge_command.active) {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_VIDEO_START_CAPTURE,
                                    MAV_RESULT_TEMPORARILY_REJECTED);
        } else {
            const uint32_t request_id = _next_bridge_request_id++;
            _pending_bridge_command.active = true;
            _pending_bridge_command.request_id = request_id;
            _pending_bridge_command.command = MAV_CMD_VIDEO_START_CAPTURE;
            _pending_bridge_command.target_sysid = msg.sysid;
            _pending_bridge_command.target_compid = msg.compid;
            _pending_bridge_command.started_ms = AP_HAL::millis();
            if (!send_video_to_bridge(request_id, true)) {
                _pending_bridge_command.active = false;
                send_camera_command_ack(msg.sysid, msg.compid,
                                        MAV_CMD_VIDEO_START_CAPTURE,
                                        MAV_RESULT_FAILED);
            }
        }
        break;
    case MAV_CMD_VIDEO_STOP_CAPTURE:
        if (!_camera.recording()) {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_VIDEO_STOP_CAPTURE,
                                    MAV_RESULT_ACCEPTED);
            send_camera_capture_status();
        } else if (_pending_bridge_command.active) {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_VIDEO_STOP_CAPTURE,
                                    MAV_RESULT_TEMPORARILY_REJECTED);
        } else {
            const uint32_t request_id = _next_bridge_request_id++;
            _pending_bridge_command.active = true;
            _pending_bridge_command.request_id = request_id;
            _pending_bridge_command.command = MAV_CMD_VIDEO_STOP_CAPTURE;
            _pending_bridge_command.target_sysid = msg.sysid;
            _pending_bridge_command.target_compid = msg.compid;
            _pending_bridge_command.started_ms = AP_HAL::millis();
            if (!send_video_to_bridge(request_id, false)) {
                _pending_bridge_command.active = false;
                send_camera_command_ack(msg.sysid, msg.compid,
                                        MAV_CMD_VIDEO_STOP_CAPTURE,
                                        MAV_RESULT_FAILED);
            }
        }
        break;
    case MAV_CMD_SET_CAMERA_MODE:
        if (isfinite(cmd.param2) &&
            (is_equal(cmd.param2, (float)CAMERA_MODE_IMAGE) ||
             is_equal(cmd.param2, (float)CAMERA_MODE_VIDEO))) {
            _camera.set_mode((uint8_t)cmd.param2);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_ACCEPTED);
            send_camera_settings();
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_DENIED);
        }
        break;
    case MAV_CMD_SET_CAMERA_ZOOM:
        if ((uint32_t)cmd.param1 == ZOOM_TYPE_RANGE) {
            _camera.set_zoom_pct(cmd.param2);
            send_zoom_to_bridge(cmd.param2);
            ::printf("MAVLinkCamV2[compid=%u]: zoom set to %.1f%%\n",
                     (unsigned)_camera_compid, (double)cmd.param2);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_ZOOM, MAV_RESULT_ACCEPTED);
            send_camera_settings();
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_ZOOM, MAV_RESULT_DENIED);
        }
        break;
    case MAV_CMD_SET_CAMERA_FOCUS:
        if ((uint32_t)cmd.param1 == FOCUS_TYPE_RANGE) {
            _camera.set_focus_pct(cmd.param2);
            ::printf("MAVLinkCamV2[compid=%u]: focus set to %.1f%%\n",
                     (unsigned)_camera_compid, (double)cmd.param2);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_FOCUS, MAV_RESULT_ACCEPTED);
            send_camera_settings();
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_FOCUS, MAV_RESULT_DENIED);
        }
        break;
    default:
        send_camera_command_ack(msg.sysid, msg.compid, (MAV_CMD)cmd.command,
                                MAV_RESULT_UNSUPPORTED);
        break;
    }
}

void MAVLinkCamV2::send_camera_heartbeat()
{
    mavlink_heartbeat_t hb {};
    hb.type            = MAV_TYPE_CAMERA;
    hb.autopilot       = MAV_AUTOPILOT_INVALID;
    hb.system_status   = MAV_STATE_ACTIVE;
    hb.mavlink_version = 3;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &hb);
    camera_send_mavlink_message(msg);
}

void MAVLinkCamV2::send_camera_information(uint32_t target_sysid, uint8_t target_compid)
{
    mavlink_camera_information_t info {};
    info.time_boot_ms     = AP_HAL::millis();
    info.firmware_version = get_camera_firmware_version();
    info.focal_length     = NAN;
    info.sensor_size_h    = NAN;
    info.sensor_size_v    = NAN;
    info.resolution_h     = get_camera_resolution_h();
    info.resolution_v     = get_camera_resolution_v();
    info.flags            = get_camera_cap_flags();
    info.gimbal_device_id = get_camera_gimbal_device_id();
    get_camera_definition(info);
    strncpy_noterm((char *)info.vendor_name, get_camera_vendor_name(), sizeof(info.vendor_name));
    strncpy_noterm((char *)info.model_name,  get_camera_model_name(),  sizeof(info.model_name));

    mavlink_message_t msg;
    mavlink_msg_camera_information_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &info);
    camera_send_mavlink_message(msg);
}

void MAVLinkCamV2::send_camera_settings()
{
    mavlink_camera_settings_t settings {};
    settings.time_boot_ms = AP_HAL::millis();
    settings.mode_id      = _camera.mode();
    settings.zoomLevel    = _camera.zoom_pct();
    settings.focusLevel   = _camera.focus_pct();

    mavlink_message_t msg;
    mavlink_msg_camera_settings_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &settings);
    camera_send_mavlink_message(msg);
}

void MAVLinkCamV2::send_camera_capture_status()
{
    mavlink_camera_capture_status_t status {};
    status.time_boot_ms = AP_HAL::millis();
    status.video_status = _camera.recording() ? 1 : 0;
    status.recording_time_ms = _camera.recording() ?
        status.time_boot_ms - _recording_started_ms : 0;
    status.available_capacity = NAN;
    status.image_count = _camera.shot_count();

    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &status);
    camera_send_mavlink_message(msg);
}

bool MAVLinkCamV2::send_video_stream_information(uint8_t stream_id)
{
    mavlink_video_stream_information_t info {};
    if (stream_id == 0 || stream_id > get_video_stream_count() ||
        !get_video_stream_information(stream_id, info)) {
        return false;
    }
    info.stream_id = stream_id;
    info.count = get_video_stream_count();

    mavlink_message_t msg;
    mavlink_msg_video_stream_information_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &info);
    camera_send_mavlink_message(msg);
    return true;
}

void MAVLinkCamV2::send_camera_command_ack(uint32_t target_sysid, uint8_t target_compid,
                                            MAV_CMD cmd, MAV_RESULT result)
{
    // pack rather than encode so a target over 255 goes in the extended
    // header rather than being truncated in the payload
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg,
        (uint16_t)cmd, (uint8_t)result,
        255,                // progress
        0,                  // result_param2
        target_sysid, target_compid);
    camera_send_mavlink_message(msg);
}

}  // namespace SITL

#endif  // AP_SIM_MAVLINKCAMV2_ENABLED
