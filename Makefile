NPM ?= npm
HOST ?= 127.0.0.1
PORT ?= 3000

.PHONY: help install dev typecheck build check serve version translations clean

help:
	@printf '%s\n' \
	  'make install    Install the locked Node.js dependencies with npm ci' \
	  'make dev        Start the editable local preview (HOST/PORT can be overridden)' \
	  'make typecheck  Check the Docusaurus TypeScript configuration' \
	  'make build      Type-check and create the production site in build/' \
	  'make check      Run every local pre-commit check' \
	  'make serve      Build and serve the production site locally' \
	  'make version VERSION=x.y  Freeze the current handbook as a release version' \
	  'make translations LOCALE=en  Generate translation messages for a configured locale' \
	  'make clean      Remove Docusaurus caches and generated site files'

install:
	$(NPM) ci

dev:
	$(NPM) run start -- --host $(HOST) --port $(PORT)

typecheck:
	$(NPM) run typecheck

build: typecheck
	$(NPM) run build

check: build
	python3 scripts/check-built-links.py build
	python3 scripts/check-legacy-urls.py build
	test -s build/search-index.json

serve: build
	$(NPM) run serve -- --host $(HOST) --port $(PORT)

version:
	@test -n "$(VERSION)" || { echo 'VERSION is required, for example: make version VERSION=0.2'; exit 2; }
	$(NPM) run docs:version -- $(VERSION)

translations:
	@test -n "$(LOCALE)" || { echo 'LOCALE is required, for example: make translations LOCALE=en'; exit 2; }
	$(NPM) run write-translations -- --locale $(LOCALE)

clean:
	$(NPM) run clear
