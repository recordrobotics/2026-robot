# Configuration file for the Sphinx documentation builder.

# -- Project information

project = "2025 Control"
copyright = "2025, Record Robotics"
author = "Record Robotics"

release = "1.0"
version = "1.0.0"

# -- General configuration

extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.doctest",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "notfound.extension",
]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
}
intersphinx_disabled_domains = ["std"]

templates_path = ["_templates"]

# -- Options for HTML output

html_theme = "sphinx_book_theme"

html_theme_options = {
    "repository_url": "https://github.com/recordrobotics/2025_Control",
    "use_repository_button": True,
    "home_page_in_toc": True,
    "path_to_docs": "docs/source/",
    "use_source_button": True,
    "use_edit_page_button": True,
    "use_issues_button": True,
    "use_fullscreen_button": False,
}

html_title = "2025 Control"

# -- Options for EPUB output
epub_show_urls = "footnote"
