import os
import sys
import json
import inspect
import subprocess
import numpy as np
import pytest

# current working directory
cwd = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))


def load_notebook(file_obj):
    """
    Load an ipynb file into a cleaned and stripped string that can
    be ran with `exec`

    The motivation for this is to check ipynb examples with CI so
    they don't get silently broken and confusing.

    Arguments
    ----------
    file_obj :  open file object

    Returns
    ----------
    script : str
      Cleaned script which can be passed to exec
    """
    raw = json.load(file_obj)
    lines = list()
    for cell in raw['cells']:
        if cell['cell_type'] == 'code':
            for line in cell['source']:
                if '#' == line[0]:
                    continue
                lines.append(line)            
    script = exclude_calls(lines)
    return script


def exclude_calls(
        lines,
        exclude=['%matplotlib',
                 '%pylab',
                 'show',
                 'plt',
                 'save_image',
                 '?']):
    """
    Exclude certain calls based on substrings, replacing
    them with pass statements.

    Parameters
    -------------
    lines : (n, ) str
      Lines making up a Python script
    exclude (m, ) str
      Substrings to exclude lines based off of

    Returns
    -------------
    joined : str
      Lines combined with newline
    """
    result = []
    for line in lines:
        # skip lines that only have whitespace or comments
        strip = line.strip()
        if len(strip) == 0 or strip.startswith('#'):
            continue
        # if the line has a blacklisted phrase switch it with a pass statement
        # we don't want to exclude function definitions however
        if not strip.startswith('def ') and any(i in line for i in exclude):
            # switch statement with pass
            line_modified = to_pass(line)
        else:
            # remove trailing whitespace
            line_modified = line.rstrip()
        # skip duplicate lines
        if len(result) > 0 and line_modified == result[-1]:
            continue
        # append the modified line to the result
        result.append(line_modified)
    # recombine into string and add trailing newline
    result = '\n'.join(result) + '\n'
    return result


def to_pass(line):
    """
    Replace a line of code with a pass statement, with
    the correct number of leading spaces

    Arguments
    ----------
    line : str, line of code

    Returns
    ----------
    passed : str, line of code with same leading spaces
                  but code replaced with pass statement
    """
    # the number of leading spaces on the line
    spaces = len(line) - len(line.lstrip(' '))
    # replace statement with pass and correct leading spaces
    passed = (' ' * spaces) + 'pass'
    return passed


def render_notebook(file_name, out_name, nbconvert='jupyter'):
    """
    Render an IPython notebook to an HTML file.
    """
    out_name = os.path.abspath(out_name)
    file_name = os.path.abspath(file_name)

    command = [nbconvert,
               'nbconvert',
               '--to',
               'html',
               file_name,
               '--output',
               out_name]

    subprocess.check_call(command)


def render_examples(out_dir, in_dir=None, ext='ipynb'):
    """
    Render all IPython notebooks in a directory to HTML.
    """
    # replace with relative path
    if in_dir is None:
        in_dir = os.path.abspath(
            os.path.join(cwd, '../examples'))

    for file_name in os.listdir(in_dir):
        # check extension
        split = file_name.split('.')
        if split[-1] != ext:
            continue
        # full path of file
        nb_path = os.path.join(in_dir, file_name)

        html_path = os.path.join(out_dir,
                                 '.'.join(split[:-1]) + '.html')
        render_notebook(nb_path, html_path)

@pytest.mark.script_launch_mode('subprocess')
def test_notebooks(script_runner):
    
    examples_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..',
        'examples'
    )

    for item in os.listdir(examples_dir): 
        filename = os.path.join(examples_dir, item)
        
        if not os.path.exists(filename):
            return

        if filename.lower().endswith('.ipynb'):               
            with open(filename, 'r') as file_obj:
                script = load_notebook(file_obj)      
            exec(script, globals())
        elif filename.lower().endswith('.py'):
            output = script_runner.run('python', filename)
            assert output.success
        else:
            continue        
