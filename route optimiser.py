#!/usr/bin/env python
# coding: utf-8

# In[1]:


import streamlit as st
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
import folium
from streamlit_folium import st_folium
import time

# Geocode addresses
def geocode_addresses(addresses):
    geolocator = Nominatim(user_agent="bronx_aps_optimizer")
    coordinates = []
    for address in addresses:
        location = None
        retries = 5
        while location is None and retries > 0:
            try:
                location = geolocator.geocode(address)
                time.sleep(1)
            except:
                time.sleep(2)
            retries -= 1
        if location:
            coordinates.append((location.latitude, location.longitude))
        else:
            st.error(f"❌ Could not geocode: {address}")
            return None
    return coordinates

# Create distance matrix
def create_distance_matrix(locations):
    n = len(locations)
    matrix = []
    for i in range(n):
        row = []
        for j in range(n):
            if i == j:
                row.append(0)
            else:
                dist = geodesic(locations[i], locations[j]).meters
                row.append(int(dist))
        matrix.append(row)
    return matrix

# Solve TSP
def solve_tsp(distance_matrix, start_index=0):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, start_index)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_params)

    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        return route
    else:
        return None

# Visualize route on folium map
def visualize_route(coords, addresses, route_indices):
    m = folium.Map(location=coords[route_indices[0]], zoom_start=13)
    for i, idx in enumerate(route_indices):
        folium.Marker(coords[idx], popup=f"{i+1}. {addresses[idx]}").add_to(m)
    folium.PolyLine([coords[i] for i in route_indices], color="blue", weight=4).add_to(m)
    return m

# Streamlit UI
st.title("🚐 Public Transit Route Optimizer")

st.markdown("Enter the starting address and client addresses below.")

start_address = st.text_input("Starting address", "Your start location here")

client_addresses_text = st.text_area(
    "Client addresses (one per line)",
    value="""3202 BRONXWOOD AVENUE, BRONX, NY
275 WEST 238th ST, BRONX, NY
2720 GRAND CONCOURSE, BRONX, NY
14 WEST 184th ST, BRONX, NY
1601 MACOMBS ROAD, BRONX, NY"""
)

if st.button("Optimize Route"):
    client_addresses = [line.strip() for line in client_addresses_text.strip().split("\n") if line.strip()]
    addresses = [start_address] + client_addresses

    with st.spinner("Geocoding addresses..."):
        coords = geocode_addresses(addresses)
    if coords is None:
        st.stop()

    with st.spinner("Creating distance matrix..."):
        matrix = create_distance_matrix(coords)

    with st.spinner("Solving optimal route..."):
        route_indices = solve_tsp(matrix, start_index=0)

    if route_indices:
        st.success("✅ Optimized Visit Order:")
        total_distance = 0
        for i in range(len(route_indices) - 1):
            from_idx = route_indices[i]
            to_idx = route_indices[i + 1]
            segment_distance = matrix[from_idx][to_idx]
            total_distance += segment_distance
            label = " (Start)" if from_idx == 0 else ""
            st.write(f"{i+1}. {addresses[from_idx]}{label} → {addresses[to_idx]} ({segment_distance:.0f} m)")

        st.write(f"**Total Distance:** {total_distance / 1000:.2f} km")

        m = visualize_route(coords, addresses, route_indices)
        st_folium(m, width=700, height=500)
    else:
        st.error("⚠️ Could not compute an optimal route.")


# In[ ]:




