from typing import Tuple

from uagents import Context

from aca_protocols.station_query_protocol import (
    StationQueryRequest,
    StationQueryResponse,
)

from aca_protocols.car_register_protocol import CarRegisterRequest, CarRegisterResponse

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData,
)

from aca_protocols.acs_registry_id import acs_id

from .fetchAgent import agent
from .filter_stations import (
    initialize_stations_properties_map,
    initialize_car_properties,
    set_PropertyData_of_sender,
    is_finished_collecting_properties,
    filter_stations,
)


def init(ctx: Context):
    initialize_car_properties(ctx)


@agent.on_message(StationQueryResponse)
async def on_is_registered(ctx: Context, sender: str, msg: StationQueryResponse):
    ctx.logger.info(f"stations: ${msg}")

    initialize_stations_properties_map(ctx, msg.stations)

    for station in msg.stations:
        ctx.logger.info(f"Requesting Properties of station: ${station}")
        await ctx.send(station, PropertyQueryRequest())


@agent.on_message(PropertyQueryResponse)
async def on_properties(ctx: Context, sender: str, msg: PropertyQueryResponse):
    ctx.logger.info(f"properties of ${sender}: ${msg}")

    set_PropertyData_of_sender(ctx, sender, msg.properties)

    if is_finished_collecting_properties(ctx):
        optimal_station: (str, PropertyData, Tuple[int, int]) = filter_stations(ctx)

        await register_at_station(ctx, optimal_station[0])


async def fetch_stations(ctx: Context):
    car_geo_point = ctx.storage.get("geo_point")

    await ctx.send(
        acs_id,
        StationQueryRequest(lat=car_geo_point[0], long=car_geo_point[1], radius=5.0),
    )


@agent.on_message(CarRegisterResponse)
async def on_registered_at_station(ctx: Context, sender: str, msg: CarRegisterResponse):
    ctx.logger.info(f"registered state: ${msg}")


async def register_at_station(ctx: Context, station: str):
    await ctx.send(
        station,
        CarRegisterRequest(start_time=0, duration=10),
    )
