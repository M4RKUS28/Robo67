"""Smoke test: the package and its pure-logic lib package import cleanly."""


def test_import_package():
    import robo67_insertion  # noqa: F401
    import robo67_insertion.lib  # noqa: F401
