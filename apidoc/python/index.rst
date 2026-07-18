Robonix Python API
==================

- **robonix_api** — the Python SDK integrators write against: primitive /
  service / skill providers, the atlas client, capability lifecycle, and
  typed channels.
- **scene_service** — the Python implementation of the scene / semantic-map
  service (perception ingest, scene graph, MCP tools). Its *consumer* API is
  the ``robonix/system/scene/*`` contracts; this is the internal reference.

.. toctree::
   :maxdepth: 2

   public-api

.. autosummary::
   :toctree: _autosummary
   :recursive:

   robonix_api
   scene_service
