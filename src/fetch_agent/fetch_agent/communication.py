import asyncio
import datetime
from xmlrpc.client import DateTime

from uagents import Context

from aca_protocols.station_query_protocol import (
    StationQueryRequest,
    StationQueryResponse,
)

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse
)

from aca_protocols.acs_registry_id import acs_id

from .fetchAgent import agent


def init(ctx: Context):
    if ctx.storage.get("car_properties") is None:
        ctx.storage.set("car_properties", PropertyQueryResponse(
            open_time_frames=[
                (datetime.datetime.now() + datetime.timedelta(hours=1)).timestamp(),
                (datetime.datetime.now() + datetime.timedelta(hours=4)).timestamp(),
            ],  # Default value: +- 4 hours from now is a possible Timeframe
            geo_point=(1, 1),
            cost_per_kwh=0.43,
            charging_wattage=11,
            green_energy=True
        ))
    if ctx.storage.get("station_to_property_map") is None:
        ctx.storage.set("station_to_property_map", {})


@agent.on_message(StationQueryResponse)
async def on_station_query_response(ctx: Context, sender: str, msg: StationQueryResponse):
    ctx.logger.info(f"ACS-Registry ({sender}): stations: ${msg}")

    loop = asyncio.get_event_loop()
    loop.run_until_complete(build_station_map_and_send_property_requests(ctx, msg.stations))
    loop.close()


@agent.on_message(PropertyQueryResponse)
async def on_property_query_response(ctx: Context, sender: str, msg: PropertyQueryResponse):
    ctx.logger.info(f"AC-Station ({sender}): properties: ${msg}")
    station_to_property_map = ctx.storage.get("station_to_property_map")
    station_to_property_map[sender] = msg

    ctx.storage.set("station_to_property_map", station_to_property_map)

    if not all(station_to_property_map.values()):
        ctx.logger.info("Accessing function for filtering stations")
        pass


async def fetch_stations(ctx: Context):
    ctx.logger.info("Fetching stations")
    await ctx.send(
        acs_id,
        StationQueryRequest(lat=1.0, long=1.0, radius=5.0),
    )


async def build_station_map_and_send_property_requests(ctx: Context, stations: list[str]):
    ctx.logger.info("Starting building property map and requesting properties")
    station_to_property_map = {}
    for station in stations:
        station_to_property_map[station] = None

        await ctx.send(
            station,
            PropertyQueryRequest()
        )

    ctx.storage.set("station_to_property_map", station_to_property_map)


def filter_stations(ctx: Context, station_to_property_map: dict[str, PropertyQueryResponse]):
    car_properties: PropertyQueryResponse = ctx.storage.get("car_properties")
    station_to_property_map = sorted(station_to_property_map,
                                     key=lambda item: sort_by_open_timeframe(car_properties.open_time_frames,
                                                                             item.open_time_frames))

    station_to_property_map = sorted(station_to_property_map,
                                     key=lambda item: sort_by_distance(car_properties.geo_point,
                                                                       item.geo_point))


def sort_by_open_timeframe(car_timeframe: list[(int, int)], station_timeframe: list[(int, int)]) -> int:
    # todo
    pass


def sort_by_distance(car_geo_point, station_geo_point) -> int:
    # todo when pathfinding is implemented
    pass


def sort_by_cost(car_cost: float, station_cost: float) -> float:
    return abs(car_cost - station_cost)


def sort_by_charging_wattage(car_cw: int, station_cw: int) -> int:
    return abs(car_cw - station_cw)


def sort_by_green_energy(car_ge: bool, station_ge: bool) -> int:
    if car_ge == True and station_ge == True:
        return 1
    elif car_ge == True and station_ge == False:
        return -1
    else:
        return 0
