from pathlib import Path

from robonix_api import Err, Ok, Primitive, Service, Skill


root = Path(__file__).parent
camera = Primitive(
    id="front_camera",
    namespace="robonix/primitive/camera",
    pkg_root=root,
)
service = Service(
    id="my_service",
    namespace="robonix/service/example",
    pkg_root=root,
)
skill = Skill(
    id="say_hello",
    namespace="robonix/skill/say_hello",
    pkg_root=root,
)


@service.on_init
def init(cfg: dict):
    return Err("params_file is required") if not cfg.get("params_file") else Ok()


@service.on_activate
def activate():
    return Ok()


@service.on_deactivate
def deactivate():
    return Ok()


@service.on_shutdown
def shutdown():
    return None


assert camera.id == "front_camera"
assert camera.namespace == "robonix/primitive/camera"
assert service._on_init({}).message == "params_file is required"
assert isinstance(service._on_init({"params_file": "config.yaml"}), Ok)
assert isinstance(service._on_activate(), Ok)
assert isinstance(service._on_deactivate(), Ok)
assert service._on_shutdown() is None
assert skill.id == "say_hello"
print("primitive/service/skill construction and lifecycle decorators: passed")
