import sys
import os

LINE_LENGTH = 42
BND_LENGTH = 1

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')


def write_line(line : str, end='\n'):
    sys.stdout.write('{:^{width}}'.format(line, width=LINE_LENGTH) + end)


def write_bounded(line, end='\n', bnd='*', aln='^', fill='', delete=True):

    if delete:
        sys.stdout.write('\r')

    # format line length
    line = '{:{fill}{align}{width}}'.format(line, width=LINE_LENGTH, align=aln, fill=fill)
    sys.stdout.write(
        '{:<{width}}'.format(bnd, width=BND_LENGTH) + ' '
        + line
        + ' ' + '{:>{width}}'.format(bnd[::-1], width=BND_LENGTH)
        + end
    )


def write_section(section):
    write_bounded(section, bnd='*', fill='-')

def write_task(task):
    write_bounded('')
    write_bounded('-> '+ task, bnd='*', aln='<')


def init_msg(msg):
    # clear_terminal()

    write_bounded('{:{fill}^{width}}'.format('', fill='-', width=LINE_LENGTH),bnd='*', fill='-')
    write_bounded(msg)
    write_bounded('{:}'.format(''))


def end_msg(msg):
    write_bounded('{:}'.format(''))
    write_bounded(msg)
    write_bounded('{:{fill}^{width}}'.format('', fill='-', width=LINE_LENGTH), bnd='*')


def write_exe_duration(time, label='total'):
    # format output
    time_msg = 'Duration: {:4.2f} seconds ({})'.format(time, label)
    # write output
    write_bounded(time_msg, bnd='*', aln='<')


def write_current_trial(trial, total):
    write_bounded('Progress: {:^4d} of {:^4d}'.format(trial,total), aln='<', bnd='*', end='', delete=True)


def write_current_id(id):
    write_bounded('Running ID: {}'.format(id), aln='<', bnd='*', end='', delete=True)

