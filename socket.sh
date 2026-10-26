set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "$DIR/venv/pump-venv/bin/python" "$DIR/server.py" "$@"
