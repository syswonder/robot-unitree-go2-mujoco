# Upstream Provenance

Copied from https://github.com/syswonder/skill-explore-rbnx at commit
`40bdfa8d143e5e7c28909992c63682111d8371dc` on 2026-09-08.

Upstream declares MulanPSL-2.0 in its package manifest, README and source SPDX
headers; those declarations are retained. That commit contains no standalone
LICENSE file. Original maintainer attribution remains in the package manifest.

Deployment modifications select connected known-free footprint-safe approaches,
suppress failed goal neighbourhoods temporarily, and serialize cancellation in
the task worker. The `explore` runtime identity and all capability TOML/IDL files
are unchanged. Git metadata, generated builds, logs and credentials were excluded
from the copy. Cached upstream repositories are not modified.
