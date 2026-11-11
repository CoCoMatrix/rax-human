"""Robot backends behind the agent MCP. The frontend never imports a backend directly.

A backend is constructed once per session by the frontend process (the only robot
owner) and implements:

    tools() -> list[dict]            MCP tool definitions (name, description, inputSchema)
    call(name, arguments) -> (value, images)
                                     value: JSON-safe dict; images: MCP image content
                                     blocks {'type': 'image', 'mimeType', 'data'(base64)}
    status() -> dict                 live robot summary merged into the `status` tool
    close()                          release hardware; must be safe to call twice

Optional: ``mutating`` (set of tool names that can move hardware; a failure in one
marks control uncertain) and ``validate(name, arguments)`` (raise before any motion).
"""
from importlib import import_module

# registry key -> "module:Class"; the module is imported only when selected.
BACKENDS = {
    'piper': 'services.piper.backend:PiperBackend',
    'franka': 'services.franka.backend:FrankaBackend',
    'mock': 'services.mock_backend:MockBackend',
}


def backend_class(name):
    try:
        module, _, attribute = BACKENDS[name].partition(':')
    except KeyError:
        raise ValueError(f"Unknown robot backend {name!r}; choose one of {', '.join(sorted(BACKENDS))}") from None
    return getattr(import_module(module), attribute)


def make_backend(config):
    """The configured robot's backend, constructed with the operator configuration."""
    return backend_class(config.robot)(config)


# Dashboard adapters are read-only and never construct a motion backend.
TELEMETRY = {
    'franka': 'services.franka.telemetry:Telemetry',
    'piper': 'services.piper.telemetry:Telemetry',
}


def make_telemetry(name):
    if name not in TELEMETRY:
        return None
    module, _, attribute = TELEMETRY[name].partition(':')
    return getattr(import_module(module), attribute)()
