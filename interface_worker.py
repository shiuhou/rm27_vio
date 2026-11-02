"""Synthetic JSONL interface endpoint. No hardware, backend estimator or training is started."""
import contextlib
import importlib.metadata
import json
from pathlib import Path
import sys

def identity():
    modules = {n: str(Path(m.__file__).resolve()) for n, m in sys.modules.items()
               if getattr(m, "__file__", None) and (n == "interface_worker" or n.startswith(("rm27.", "rm27_contact_")))}
    versions = {}
    for name in ("numpy", "scipy", "mujoco", "pymavlink", "torch", "gymnasium", "stable-baselines3"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    return {"source": SOURCE, "worker": str(Path(__file__).resolve()), "python": sys.executable,
            "python_version": sys.version, "modules": modules, "versions": versions,
            "qualification": "SYNTHETIC_INTERFACE_ONLY"}

SOURCE = "vio"

from rm27.perception.vision.localization.schema import LocalizationEstimate

def handle(kind, p):
    if kind == "hello": return identity()
    if kind == "localization":
        # No promotion of unknown raw backend evidence; preserve every native field.
        return LocalizationEstimate.from_dict(p).to_dict()
    raise ValueError("unsupported kind")

def main():
    previous = -1
    for line in sys.stdin:
        request = None
        try:
            request = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
            if (type(request.get("version")) is not int or request.get("version") != 1 or request.get("source") != "coordinator"
                    or type(request.get("seq")) is not int or request["seq"] <= previous
                    or request.get("input_kind") != "synthetic"):
                raise ValueError("invalid envelope/source/sequence/input_kind")
            previous = request["seq"]
            with contextlib.redirect_stdout(sys.stderr):
                result = handle(request["kind"], request["payload"])
            reply = {"version": 1, "source": SOURCE, "seq": request["seq"],
                     "input_kind": "synthetic", "kind": request["kind"], "ok": True, "payload": result}
        except Exception as exc:
            reply = {"version": 1, "source": SOURCE, "seq": request.get("seq") if isinstance(request, dict) else None,
                     "input_kind": "synthetic", "kind": request.get("kind") if isinstance(request, dict) else None,
                     "ok": False, "error": str(exc), "error_type": type(exc).__name__}
        print(json.dumps(reply, allow_nan=False), flush=True)

if __name__ == "__main__":
    main()
