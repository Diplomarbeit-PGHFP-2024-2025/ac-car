import asyncio
import os
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
from .sort_stations import (
    initialize_stations_properties_map,
    set_PropertyData_of_sender,
    sort_stations,
)

from dotenv import load_dotenv
from aca_protocols.ac_payment_protocol import PaymentRequest, TransactionInfo
from aca_protocols.ac_charging_protocol import CarFinishedChargingInfo

load_dotenv()

optimal_station_future = asyncio.Future()


@agent.on_message(StationQueryResponse)
async def on_is_registered(ctx: Context, sender: str, msg: StationQueryResponse):
    asyncio.ensure_future(on_station_query_response(ctx, msg))


async def on_station_query_response(ctx: Context, msg: StationQueryResponse):
    ctx.logger.info(f"stations: {msg}")
    initialize_stations_properties_map(ctx)

    for station in msg.stations:
        ctx.logger.info(f"Requesting Properties of station: {station}")
        await ctx.send(station, PropertyQueryRequest())

    await _wait_for_stations(ctx)

    optimal_station: (str, PropertyData, Tuple[int, int]) = sort_stations(ctx)
    optimal_station_future.set_result(optimal_station)


async def _wait_for_stations(ctx):
    waiting_time = int(os.getenv("WAITING_TIME"))
    ctx.storage.set("finished_waiting", False)
    ctx.logger.info(f"Waiting for responses from stations for: {waiting_time} seconds")
    await asyncio.sleep(waiting_time)
    ctx.storage.set("finished_waiting", True)


@agent.on_message(PropertyQueryResponse)
async def on_properties(ctx: Context, sender: str, msg: PropertyQueryResponse):
    ctx.logger.info(f"properties of {sender}: {msg}")

    set_PropertyData_of_sender(ctx, sender, msg.properties)


async def fetch_stations(
    ctx: Context, car_geo_point: tuple[float, float]
) -> (str, PropertyData, Tuple[int, int]):
    global optimal_station_future
    optimal_station_future = asyncio.Future()

    await ctx.send(
        acs_id,
        StationQueryRequest(lat=car_geo_point[0], long=car_geo_point[1], radius=5.0),
    )

    return await optimal_station_future


@agent.on_message(CarRegisterResponse)
async def on_registered_at_station(ctx: Context, sender: str, msg: CarRegisterResponse):
    ctx.logger.info(f"registered state: {msg}")

    await ctx.send(sender, CarFinishedChargingInfo(kwh_charged=12))


async def register_at_station(ctx: Context, station: str, time_frame: Tuple[int, int]):
    await ctx.send(
        station,
        CarRegisterRequest(
            start_time=time_frame[0], duration=time_frame[1] - time_frame[0]
        ),
    )


@agent.on_message(PaymentRequest)
async def on_payment_requested(ctx: Context, sender: str, msg: PaymentRequest):
    ctx.logger.info(f"[Payment, on_payment_request]: sender: {sender}, msg: {msg}")

    transaction = ctx.ledger.send_tokens(
        msg.wallet_address, msg.amount, msg.denomination, agent.wallet
    )

    await ctx.send(sender, TransactionInfo(transaction_hash=transaction.tx_hash))
