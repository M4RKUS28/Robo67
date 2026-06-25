import { useQuery } from "@tanstack/react-query";
import type { Config, Health } from "./types";

async function getJSON<T>(url: string): Promise<T> {
  const res = await fetch(url);
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
