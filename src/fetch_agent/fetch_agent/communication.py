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
                (datetime.datetime.now() - datetime.timedelta(hours=2)).timestamp(),
                (datetime.datetime.now() + datetime.timedelta(hours=2)).timestamp(),
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


def filter_stations(ctx: Context, station_to_property_map: dict[str, PropertyQueryResponse]) -> bool:
    car_properties: PropertyQueryResponse = ctx.storage.get("car_properties")
    for e in station_to_property_map.values():

        if car_properties.green_energy == True & e.green_energy != True:
            return False


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
