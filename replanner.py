#!/usr/bin/env python3
"""Lightweight replanner module for Stage-2.
Provides traffic-aware shortest-path (Dijkstra) with configurable weights.
"""
import math
import heapq

def dijkstra_weighted(adj, nodes, counts, start, target, alpha=1.0, beta=2.0):
    N = len(nodes)
    dist = [math.inf]*N
    prev = [-1]*N
    dist[start]=0
    pq=[(0,start)]
    maxc = max(1, max(counts))
    while pq:
        d,u = heapq.heappop(pq)
        if d!=dist[u]:
            continue
        if u==target:
            break
        for v,w in adj.get(u,[]):
            traffic = (counts[u] + counts[v]) / 2.0
            nd = d + alpha*w + beta*(traffic / maxc)
            if nd < dist[v]:
                dist[v]=nd; prev[v]=u
                heapq.heappush(pq,(nd,v))
    # reconstruct
    if math.isinf(dist[target]):
        return None
    path = []
    u = target
    while u!=-1:
        path.append(u)
        if u==start: break
        u = prev[u]
    return list(reversed(path))

__all__ = ['dijkstra_weighted']
