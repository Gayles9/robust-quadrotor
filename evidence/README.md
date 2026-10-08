# Reproducibility evidence

The [results guides](../docs/results/README.md) explain the methods, measured
outcomes and limitations. This directory holds supporting reference material
for inspecting the evidence behind those explanations.

## Reference records

[`development-records.zip`](development-records.zip) preserves the original
verification records, detailed numerical tables, source identities and report
notes. The archive contains 81 documents at their original relative paths and
a `MANIFEST.json` with the byte count and SHA-256 digest of every document.
The digest identifies exact file contents, so a changed record can be detected.

These are snapshots of the recorded experiments. Their commands, environment
descriptions and results belong to those experiments. Use the
[getting-started guide](../docs/guides/getting-started.md) for current commands.

The archive contains documentation, not the full flight-history payloads.
Individual study guides identify external campaign archives required for a
complete reconstruction. The [official report](../docs/report.md) and its editable
source are also separate deliverables.

## Frozen experiment inputs

Several experiment runners authenticate the exact text that defines their cases
and acceptance conditions. Those bytes are retained under
[`experiments/protocols`](../experiments/protocols/README.md). Relative symbolic
links at the loaders' established paths preserve compatibility. The maintained
[design explanations](../docs/decisions/README.md) describe the same technical
contracts for readers.

The source fingerprints, random seeds, numerical histories and acceptance
thresholds are unchanged by this documentation organization.
