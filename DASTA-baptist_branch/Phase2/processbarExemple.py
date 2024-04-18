from __future__ import annotations

import contextlib
import functools
import os
import random
import sys
import time
import typing

import progressbar

examples: list[typing.Callable[[typing.Any], typing.Any]] = []


def example(fn):
    '''Wrap the examples so they generate readable output'''

    @functools.wraps(fn)
    def wrapped(*args, **kwargs):
        try:
            sys.stdout.write('Running: %s\n' % fn.__name__)
            fn(*args, **kwargs)
            sys.stdout.write('\n')
        except KeyboardInterrupt:
            sys.stdout.write('\nSkipping example.\n\n')
            # Sleep a bit to make killing the script easier
            time.sleep(0.2)

    examples.append(wrapped)
    return wrapped


@example
def multiple_bars_line_offset_example():
    BARS = 5
    N = 10

    bars = [
        progressbar.ProgressBar(
            max_value=N,
            # We add 1 to the line offset to account for the `print_fd`
            line_offset=i + 1,
            max_error=False,
        )
        for i in range(BARS)
    ]
    # Create a file descriptor for regular printing as well
    print_fd = progressbar.LineOffsetStreamWrapper(lines=0, stream=sys.stdout)
    assert print_fd

    # The progress bar updates, normally you would do something useful here
    for _ in range(N * BARS):
        time.sleep(0.005)

        # Increment one of the progress bars at random
        bars[random.randrange(0, BARS)].increment(0.1)

    # Cleanup the bars
    for bar in bars:
        bar.finish()
        # Add a newline to make sure the next print starts on a new line
        print()


# same but with a title for the bars
@example
def multiple_bars_line_offset_example_with_title():
    BARS = 5
    N = 10

    bars = [
        progressbar.ProgressBar(
            max_value=N,
            line_offset=i + 1,
            max_error=False, # max_error=True will show the error in red
            label=f'Bar {i}',
            append_label=True,
        )
        for i in range(BARS)
    ]
    # Create a file descriptor for regular printing as well
    print_fd = progressbar.LineOffsetStreamWrapper(lines=0, stream=sys.stdout)
    assert print_fd

    # The progress bar updates, normally you would do something useful here
    for _ in range(N * BARS):
        time.sleep(0.5)

        # Increment one of the progress bars at random
        bars[random.randrange(0, BARS)].increment(0.1)

    # Cleanup the bars
    for bar in bars:
        bar.finish()
        # Add a newline to make sure the next print starts on a new line
        print()
        

def test(*tests):
    if tests:
        no_tests = True
        for example in examples:
            for test in tests:
                if test in example.__name__:
                    example()
                    no_tests = False
                    break

        if no_tests:
            for example in examples:
                print('Skipping', example.__name__)
    else:
        for example in examples:
            example()


if __name__ == '__main__':
    try:
        test(*sys.argv[1:])
    except KeyboardInterrupt:
        sys.stdout.write('\nQuitting examples.\n')