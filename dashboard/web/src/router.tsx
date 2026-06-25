import {
  createRootRoute,
  createRoute,
  createRouter,
  Outlet,
} from "@tanstack/react-router";
import { AppShell } from "./components/AppShell";
import { Overview } from "./routes/Overview";
import { Cameras } from "./routes/Cameras";
import { Decisions } from "./routes/Decisions";

const rootRoute = createRootRoute({
  component: () => (
    <AppShell>
      <Outlet />
    </AppShell>
  ),
});

const indexRoute = createRoute({
  getParentRoute: () => rootRoute,
  path: "/",
  component: Overview,
});

const camerasRoute = createRoute({
  getParentRoute: () => rootRoute,
  path: "/cameras",
  component: Cameras,
});

const decisionsRoute = createRoute({
  getParentRoute: () => rootRoute,
  path: "/decisions",
  component: Decisions,
});

const routeTree = rootRoute.addChildren([indexRoute, camerasRoute, decisionsRoute]);

export const router = createRouter({ routeTree, defaultPreload: "intent" });

declare module "@tanstack/react-router" {
  interface Register {
    router: typeof router;
  }
}
