# SPDX-License-Identifier: MulanPSL-2.0
# Sphinx config for the Robonix Python API reference (robonix_api).
#
# Output is gitignored and served at /api/python; CI builds it fresh into
# build/api/python (so the book repo never carries generated HTML).
import os
import sys

# This configuration lives in the standalone Book repository, not under the
# Robonix source tree. Require explicit source roots so a misplaced command
# cannot silently publish empty autosummary pages.


def _add(p):
    if p and os.path.isdir(p):
        sys.path.insert(0, p)


_api_src = os.environ.get("ROBONIX_API_SRC", "")
_scene = os.environ.get("ROBONIX_SCENE_SRC", "")
if not os.path.isdir(_api_src) or not os.path.isdir(_scene):
    raise RuntimeError(
        "Set ROBONIX_API_SRC and ROBONIX_SCENE_SRC to directories in the "
        "pinned Robonix source checkout; see docs/reference/api.md."
    )

# robonix_api SDK.
_add(_api_src)
# scene service + its codegen-generated stubs (semantic_map_mcp / *_pb2),
# which scene_service.mcp_tools imports when available. CI and the documented
# lightweight local build mock those optional imports.
_add(_scene)
_add(os.path.join(_scene, "rbnx-build/codegen/robonix_mcp_types"))
_add(os.path.join(_scene, "rbnx-build/codegen/proto_gen"))

project = "Robonix Python API"
author = "Robonix"
release = os.environ.get("ROBONIX_DOC_VERSION", "v0.1")

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]
autosummary_generate = True
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}
napoleon_google_docstring = True
napoleon_numpy_docstring = True

# Heavy optional dependencies to mock when they are not installed. CI and the
# lightweight local procedure both pass a comma-separated module list.
autodoc_mock_imports = [
    m.strip() for m in os.environ.get("ROBONIX_SPHINX_MOCKS", "").split(",") if m.strip()
]

html_theme = "furo"
html_title = "Robonix Python API"
html_show_sphinx = False
html_static_path = ["_static"]
html_css_files = ["robonix.css"]
# Match the mdBook: robonix blue accent, Noto Sans CJK SC body, JetBrains Mono.
html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#283689",
        "color-brand-content": "#283689",
        "font-stack": '"Noto Sans CJK SC", "Noto Sans SC", -apple-system, '
        'BlinkMacSystemFont, "Segoe UI", sans-serif',
        "font-stack--monospace": '"JetBrains Mono", ui-monospace, '
        "SFMono-Regular, Menlo, Consolas, monospace",
    },
    "dark_css_variables": {
        "color-brand-primary": "#6178c8",
        "color-brand-content": "#6178c8",
    },
}
