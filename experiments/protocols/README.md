# Frozen experiment inputs

These files are immutable protocol data for the experiment runners. A protocol
defines the cases, assumptions and acceptance criteria used in a study. The
runners hash the exact file bytes to bind saved results to those conditions, so
these originals retain their complete recorded contents.

For explanations of the implemented methods and their results, use:

- [Design decisions](../../docs/decisions/README.md).
- [Supported alignment](../../docs/decisions/stationary-prearm-alignment-design.md).
- [Final controller comparison](../../docs/decisions/final-geometric-comparison.md).
- [Results and validation](../../docs/results/README.md).

The numbered filenames identify the original protocols. Relative symbolic
links at the existing `docs/decisions` and `docs/progress` input paths preserve
the experiment loaders and previously recorded protocol identities. These
links are compatibility paths, not the maintained reading path. Use a Linux or
WSL checkout that preserves symbolic links, as required by the supported
experiment workflow.

The execution source and authentication rules remain unchanged. Editing these
inputs would change the experiment identity; editorial explanations belong in
the linked documentation.
