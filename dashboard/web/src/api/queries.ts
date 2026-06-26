import { useQuery } from "@tanstack/react-query";
import type {
  BringupStatus,
  Config,
  FciStatus,
  GripperStatus,
  Health,
  HomeStatus,
  InsertionStatus,
} from "./types";

async function getJSON<T>(url: string): Promise<T> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url} -> ${res.status}`);
  return (await res.json()) as T;
}

async function postJSON<T>(url: string): Promise<T> {
  const res = await fetch(url, { method: "POST" });
  if (!res.ok) throw new Error(`${url} -> ${res.status}`);
  return (await res.json()) as T;
}

export function useHealth() {
  return useQuery({
    queryKey: ["health"],
    queryFn: () => getJSON<Health>("/api/health"),
    refetchInterval: 4000,
    retry: true,
  });
}

export function useConfig() {
  return useQuery({
    queryKey: ["config"],
    queryFn: () => getJSON<Config>("/api/config"),
    staleTime: Infinity,
  });
}

// Automated-insertion control (live mode only). Polls status; start/stop POST.
export function useInsertionStatus() {
  return useQuery({
    queryKey: ["insertion-status"],
    queryFn: () => getJSON<InsertionStatus>("/api/insertion/status"),
    refetchInterval: 1000,
    retry: true,
  });
}

export function startInsertion() {
  return postJSON<{ ok: boolean; error?: string; pid?: number }>("/api/insertion/start");
}

export function stopInsertion() {
  return postJSON<{ ok: boolean; error?: string }>("/api/insertion/stop");
}

// Arm bringup relaunch control (live mode only). Polls status; relaunch POST.
export function useBringupStatus() {
  return useQuery({
    queryKey: ["bringup-status"],
    queryFn: () => getJSON<BringupStatus>("/api/bringup/status"),
    refetchInterval: 1000,
    retry: true,
  });
}

export function relaunchBringup() {
  return postJSON<{ ok: boolean; error?: string; started?: boolean }>("/api/bringup/relaunch");
}

// "Bring to home" control (live mode only). Polls status; run/stop POST.
export function useHomeStatus() {
  return useQuery({
    queryKey: ["home-status"],
    queryFn: () => getJSON<HomeStatus>("/api/home/status"),
    refetchInterval: 1000,
    retry: true,
  });
}

export function runHome() {
  return postJSON<{ ok: boolean; error?: string; pid?: number }>("/api/home/run");
}

export function stopHome() {
  return postJSON<{ ok: boolean; error?: string }>("/api/home/stop");
}

// FCI on/off control (live mode only). Polls status; activate/deactivate POST.
export function useFciStatus() {
  return useQuery({
    queryKey: ["fci-status"],
    queryFn: () => getJSON<FciStatus>("/api/fci/status"),
    refetchInterval: 1000,
    retry: true,
  });
}

export function activateFci() {
  return postJSON<{ ok: boolean; error?: string; started?: boolean }>("/api/fci/activate");
}

export function deactivateFci() {
  return postJSON<{ ok: boolean; error?: string; started?: boolean }>("/api/fci/deactivate");
}

// Gripper open/close control (live mode only). Polls status; open/close POST.
export function useGripperStatus() {
  return useQuery({
    queryKey: ["gripper-status"],
    queryFn: () => getJSON<GripperStatus>("/api/gripper/status"),
    refetchInterval: 1000,
    retry: true,
  });
}

export function openGripper() {
  return postJSON<{ ok: boolean; error?: string; started?: boolean }>("/api/gripper/open");
}

export function closeGripper() {
  return postJSON<{ ok: boolean; error?: string; started?: boolean }>("/api/gripper/close");
}
