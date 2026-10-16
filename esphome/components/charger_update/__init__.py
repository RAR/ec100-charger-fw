# charger_update: an update entity on the WiFi bridge that installs the charger's firmware from GitHub releases
# (docs/history/superpowers/specs/2026-10-16-bridge-charger-update-design.md). The platform is in update.py.
import esphome.codegen as cg

CODEOWNERS = []
charger_update_ns = cg.esphome_ns.namespace("charger_update")
