### NanoCBOR fuzz testing setup

This test provides a small application that reads a cbor structure from stdin.

First it will try to advance past the advance structure to validate the `nanocbor_skip()` function

Second it will print as much of the cbor possible.

#### Building

```
make test
```

#### Inputs

Example inputs are provided in the inputs dir
