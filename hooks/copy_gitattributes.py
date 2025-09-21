"""MkDocs hooks for documentation build."""

import os
import shutil


def on_post_build(config):
    """Copy .gitattributes file to site directory after build."""
    shutil.copy(".gitattributes", os.path.join(config["site_dir"], ".gitattributes"))
