"""``python -m dma`` → the same CLI as the ``aim2`` console script."""
from dma.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
