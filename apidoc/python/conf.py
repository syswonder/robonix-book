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
_scene_codegen = os.environ.get("ROBONIX_SCENE_CODEGEN", "")
if (
    not os.path.isdir(_api_src)
    or not os.path.isdir(_scene)
    or not os.path.isdir(_scene_codegen)
):
    raise RuntimeError(
        "Set ROBONIX_API_SRC, ROBONIX_SCENE_SRC, and ROBONIX_SCENE_CODEGEN "
        "to the pinned source and generated Scene interfaces; use the Book "
        "Makefile targets described in docs/reference/api.md."
    )

# robonix_api SDK.
_add(_api_src)
# Scene implementation plus codegen-generated semantic_map_mcp and protobuf
# modules. The Makefile generates these in a temporary directory so API
# generation never writes into the clean pinned source checkout.
_add(_scene)
_add(os.path.join(_scene_codegen, "robonix_mcp_types"))
_add(os.path.join(_scene_codegen, "proto_gen"))

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
autosummary_generate_overwrite = False
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
# Match the Docusaurus handbook: Robonix blue, Noto Sans SC, JetBrains Mono.
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
