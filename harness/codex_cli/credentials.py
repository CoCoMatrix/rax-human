"""Host-only loading. Never execute a credential file or include its contents in errors."""
import ast
import os
from pathlib import Path
from urllib.parse import urlparse


def load_credentials(path, base_url=None):
    raw = Path(path).read_text().strip()
    values = {}
    if raw.startswith(('sk-', 'sess-')) and not any(c.isspace() for c in raw):
        key = raw
    else:
        try:
            tree = ast.parse(raw)
            for node in tree.body:
                if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                    values[node.targets[0].id.lower()] = ast.literal_eval(node.value)
            key = next((values[k] for k in ('api_key', 'openai_api_key', 'apikey', 'key') if values.get(k)), None)
        except (SyntaxError, ValueError, TypeError):
            raise ValueError('Credential file must be a raw token or literal Python assignments') from None
    if not isinstance(key, str) or not key or any(c.isspace() for c in key):
        raise ValueError('Credential file contains no valid API key')
    base_url = base_url or values.get('base_url') or values.get('openai_base_url') or os.getenv('RAX_CODEX_BASE_URL')
    if not isinstance(base_url, str):
        raise ValueError('Provide --base-url or a base_url literal in the credential file')
    u = urlparse(base_url)
    if u.username or u.password or u.query or u.fragment or not u.netloc or u.scheme != 'https':
        raise ValueError('Model base URL must be HTTPS without credentials, query, or fragment')
    return key, base_url.rstrip('/')
