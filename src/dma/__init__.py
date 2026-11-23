"""dma — real-time visual aim pipeline (v2).

Public entry points:

    python -m dma.cli doctor|run|sim|bench|autotune|replay|config|calibrate

The legacy v1 modules (``aim``, ``vision``, ``predict``, ``logic``, ``main_cli``,
``qt_gui``) are still importable for rollback; they are not used by the v2 engine.
"""

__version__ = "2.0.0"
