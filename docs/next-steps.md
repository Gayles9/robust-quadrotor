# Next steps

The current simulation stack and bounded studies are described in
[results and validation](results/README.md). The original cascade remains the
default, and controller improvement pauses after the final geometric comparison.
The following items are planned work, not implemented capability.

1. Add the official report and matching source package in a separate update. Check
   the package's figures, references and build without changing scientific claims.
2. Before any further noisy-feedback or mass-mismatch work, define the mechanism
   being tested, fixed comparisons and acceptance criteria. Preserve existing
   failures and keep development cases separate from fresh qualification.
3. If middleware work resumes, first specify a simulation-only ROS 2/PX4
   interface: frames, clocks, data ownership, command authority and acceptance
   checks. The custom Python stack has no deployed adapter or hardware validation.

The supported-start assumption is explicit and does not establish a physical
arming procedure. A numerical observation-loss abort does not provide a flight
fallback. The independent-inclination extension is closed under its current
unsupported sensor assumptions.

The [capability ledger](results/operating-envelope.md) records the implemented
features and the evidence required to extend their demonstrated scope.
