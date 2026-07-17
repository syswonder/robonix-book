from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

from robonix_api import ATLAS, Deferred, Ok, Service
from robonix_api.atlas_types import Capability, Channel, Kind, Transport


navigate_service = Service(
    id="my_navigate",
    namespace="robonix/service/navigation",
    pkg_root=Path(__file__).parent,
)
move_channel: Channel | None = None


@navigate_service.on_activate
def activate():
    global move_channel
    if move_channel is not None:
        move_channel.close()
        move_channel = None
    matches = ATLAS.find_capability(
        contract_id="robonix/primitive/chassis/move",
        transport=Transport.GRPC,
        provider_id="base_chassis",
    )
    if not matches:
        return Deferred("base_chassis is not ready")
    move_channel = navigate_service.connect_capability(
        matches[0],
        "robonix/primitive/chassis/move",
        Transport.GRPC,
    )
    return Ok()


@navigate_service.on_deactivate
def deactivate():
    global move_channel
    if move_channel is not None:
        move_channel.close()
        move_channel = None
    return Ok()


@navigate_service.on_shutdown
def shutdown():
    return deactivate()


match = Capability(
    provider_id="base_chassis",
    provider_kind=Kind.PRIMITIVE,
    contract_id="robonix/primitive/chassis/move",
    transport=Transport.GRPC,
)
first_channel = Channel(
    provider_id="base_chassis",
    contract_id="robonix/primitive/chassis/move",
    transport=Transport.GRPC,
    endpoint="127.0.0.1:50052",
    channel_id="channel-1",
)
second_channel = Channel(
    provider_id="base_chassis",
    contract_id="robonix/primitive/chassis/move",
    transport=Transport.GRPC,
    endpoint="127.0.0.1:50052",
    channel_id="channel-2",
)

with patch.object(ATLAS, "find_capability", return_value=[]):
    result = navigate_service._on_activate()
    assert isinstance(result, Deferred)
    assert result.reason == "base_chassis is not ready"

with (
    patch.object(ATLAS, "find_capability", return_value=[match]),
    patch.object(
        ATLAS,
        "connect_capability",
        side_effect=[first_channel, second_channel],
    ) as connect,
):
    assert isinstance(navigate_service._on_activate(), Ok)
    assert move_channel is first_channel
    assert isinstance(navigate_service._on_activate(), Ok)
    assert first_channel._closed
    assert move_channel is second_channel
    assert connect.call_count == 2

assert isinstance(navigate_service._on_deactivate(), Ok)
assert second_channel._closed
assert move_channel is None
assert isinstance(navigate_service._on_shutdown(), Ok)

print("provider discovery, channel replacement, and lifecycle cleanup: passed")
