from uagents import Context

from aca_protocols.station_query_protocol import (
    StationQueryRequest,
    StationQueryResponse,
)

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
)

from aca_protocols.acs_registry_id import acs_id

from .fetchAgent import agent


@agent.on_message(StationQueryResponse)
async def on_is_registered(ctx: Context, sender: str, msg: StationQueryResponse):
    ctx.logger.info(f"stations: ${msg}")

    address = msg.stations[0]
    ctx.logger.info(f"Example Requesting properties from: ${address}")
    await ctx.send(address, PropertyQueryRequest())


@agent.on_message(PropertyQueryResponse)
async def on_properties(ctx: Context, sender: str, msg: PropertyQueryResponse):
    ctx.logger.info(f"properties of ${sender}: ${msg}")


async def fetch_stations(ctx: Context):
    await ctx.send(
        acs_id,
        StationQueryRequest(lat=1.0, long=1.0, radius=5.0),
    )
