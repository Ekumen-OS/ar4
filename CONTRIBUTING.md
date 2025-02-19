# Contributing

## Using pre-commit

Pre-commit is a tool that allows git's pre-commit hook integrate with various code linters and formatters.

To install `pre-commit`, run
```sh
pip install pre-commit
```

To automatically run it on each commit, from repository's root:
```sh
pre-commit install
```

And that's it! Every time you commit, `pre-commit` will trigger and let you know if everything goes well.
If the checks fail, the commit won't be created, and you'll have to fix the issue (some of them are automatically fixed by `pre-commit`), STAGE the changes, and try again.

To manually run `pre-commit` on the staged changes run:
```sh
pre-commit run
```

Or to make a pass to the whole codebase
```sh
pre-commit run --all-files
```
