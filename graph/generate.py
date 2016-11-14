# -*- coding: utf-8 -*-
from jinja2 import Environment, PackageLoader
from pathlib import Path

__all__ = [ 'create_graph_file' ]


environ = Environment(loader = PackageLoader('graph', 'templates'))
template = environ.get_template('graph-template.py.tmp')

def create_graph_file(width: int, height: int, path: Path):

    with path.open('wb') as f:

        data = template.render(width = width, height = height)
        f.write(data.encode('utf-8'))
