"""URDF to MJCF conversion package."""

__all__ = ["URDF2MJCFConverter", "main"]


def __getattr__(name: str):
    if name in {"URDF2MJCFConverter", "main"}:
        from .converter import URDF2MJCFConverter, main

        return {"URDF2MJCFConverter": URDF2MJCFConverter, "main": main}[name]
    raise AttributeError(name)
