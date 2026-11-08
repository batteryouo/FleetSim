"""Shared output-path restrictions for local tooling artifacts."""
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def fixture_output(path):
  """Returns a resolved path under tmp/ or OS temporary storage, or raises ValueError."""
  path = Path(path).resolve()
  allowed = [ROOT / "tmp", Path(tempfile.gettempdir())]
  if not any(path.is_relative_to(root.resolve()) for root in allowed):
    raise ValueError("Tool outputs must stay under tmp/ or the OS temporary directory")
  return path
