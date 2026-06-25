import { useQuery } from "@tanstack/react-query";
import type { Config, Health, InsertionStatus } from "./types";

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
