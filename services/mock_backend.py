"""A hardware-free backend for tests and launcher dry runs. It moves nothing."""


class MockBackend:
    mutating = frozenset({'mock_move'})

    def __init__(self, config):
        self.config = config
        self.position = [0.0, 0.0, 0.0]
        self.closed = False

    def tools(self):
        return [
            {'name': 'mock_observe', 'description': 'Report the simulated end-effector position.',
             'inputSchema': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False}},
            {'name': 'mock_move', 'description': 'Set the simulated end-effector position (metres).',
             'inputSchema': {'type': 'object', 'additionalProperties': False, 'required': ['position'],
                             'properties': {'position': {'type': 'array', 'items': {'type': 'number'},
                                                         'minItems': 3, 'maxItems': 3}}}},
        ]

    def validate(self, name, arguments):
        if name == 'mock_move' and any(abs(v) > 1 for v in arguments['position']):
            raise ValueError('position outside the mock workspace')

    def call(self, name, arguments):
        if name == 'mock_move':
            self.position = [float(v) for v in arguments['position']]
        return {'position': list(self.position)}, []

    def status(self):
        return {'robot': 'mock', 'position': list(self.position)}

    def close(self):
        self.closed = True
