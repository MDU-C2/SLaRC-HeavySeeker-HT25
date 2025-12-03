import type { ExtensionContext, PanelExtensionContext } from "@foxglove/studio";
import { mountPanel } from "./ClickToGoalPanel";

export function activate(ctx: ExtensionContext) {
  ctx.registerPanel({
    name: "Click-to-Goal Map",
    initPanel: (panelCtx: PanelExtensionContext) => {
      mountPanel(panelCtx);
    },
  });
}