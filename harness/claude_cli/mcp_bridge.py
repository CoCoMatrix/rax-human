"""Stdio MCP server for Claude Code that forwards to the session's robot service socket.

Claude Code spawns this as its MCP server; the robot service (services.controller.frontend
--socket) is started once by the launcher and owns the robot, so Claude Code's direct
tool calls and the agent's Python scripts share one serialized robot owner.
"""
import os
import socket
import sys
import threading


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.environ['RAX_MCP_SOCKET']
    with socket.socket(socket.AF_UNIX) as connection:
        connection.connect(path)

        def upload():
            try:
                while block := os.read(0, 65536):
                    connection.sendall(block)
            finally:
                try:
                    connection.shutdown(socket.SHUT_WR)
                except OSError:
                    pass
        threading.Thread(target=upload, daemon=True).start()
        while block := connection.recv(65536):
            sys.stdout.buffer.write(block)
            sys.stdout.buffer.flush()


if __name__ == '__main__':
    main()
