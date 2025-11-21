import { ExtensionContext } from "@foxglove/extension";

import { initStatusPanel } from "./status_panel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "status-panel", initPanel: initStatusPanel });
}
