"use client";

import { DocsSidebar } from "@/components/docs-sidebar";
import { sidebarItems } from "@/components/docs-sidebar-config";

export function DocsSidebarWrapper() {
  return <DocsSidebar items={sidebarItems} />;
}
