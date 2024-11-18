from aca_protocols import PropertyData
from typing import Tuple
from uagents import Context
import datetime

json_key_stations_properties_map: str = "stations_properties_map"


def read_stations_properties_map(ctx: Context) -> dict[str, PropertyData]:
    ctx.logger.info("[Filter Stations, read_stations_properties_map]: Starting reading")
    unserialized: dict[str, str] = {}

    if not ctx.storage.get(json_key_stations_properties_map):
        ctx.logger.info("[Filter Stations, read_stations_properties_map]: Found Stations Properties")
        unserialized = ctx.storage.get(json_key_stations_properties_map)

    ret: dict[str, PropertyData] = {}

    for k, v in unserialized.items():
        ret[k] = PropertyData.fromJson(v)

    return ret


def serialize_and_save_stations_properties_map(ctx: Context, serialized_map: dict[str, PropertyData]):  #
    ctx.logger.info("[Filter Stations, serialize_and_save_stations_properties_map]: Starting serializing and saving")
    to_save: dict[str, str] = {}

    for k, v in serialized_map.items():
        to_save[k] = v.toJson()

    ctx.storage.set(json_key_stations_properties_map, to_save)


def read_car_properties(ctx: Context) -> PropertyData:
    ctx.logger.info("[Filter Stations, read_car_properties]: Starting reading")
    return PropertyData(
        open_time_frames=ctx.storage.get("open_time_frames"),
        geo_point=ctx.storage.get("geo_point"),
        cost_per_kwh=ctx.storage.get("cost_per_kwh"),
        charging_wattage=ctx.storage.get("charging_wattage"),
        green_energy=ctx.storage.get("green_energy")
    )


def initialize_stations_properties_map(ctx: Context, senders: list[str]):
    to_save = {}

    for sender in senders:
        to_save[sender] = None

    ctx.storage.set(json_key_stations_properties_map, to_save)


def set_PropertyData_of_sender(ctx: Context, sender: str, properties: PropertyData | object):
    ctx.logger.info(f"[Filter Stations, set_PropertyData_of_sender]: Setting properties of ${sender} to ${properties}")

    stations_properties: dict[str, PropertyData] = read_stations_properties_map(ctx)

    stations_properties[sender] = properties

    serialize_and_save_stations_properties_map(ctx, stations_properties)


def is_finished_collecting_properties(ctx: Context) -> bool:
    return all(read_stations_properties_map(ctx).values())


def initialize_car_properties(ctx: Context):
    ctx.storage.set("open_time_frames", [
        (
            (
                    datetime.datetime.now() + datetime.timedelta(hours=1)
            ).timestamp(),
            (
                    datetime.datetime.now() + datetime.timedelta(hours=4)
            ).timestamp(),
        ),
    ])

    ctx.storage.set("geo_point", (20.32, 85.52))
    ctx.storage.set("cost_per_kwh", 0.43)
    ctx.storage.set("charging_wattage", 11)
    ctx.storage.set("green_energy", True)


def filter_stations(ctx: Context) -> (str, PropertyData, Tuple[int, int]):
    ctx.logger.info(f"[Filter Stations, filter_stations]: Starting to filter")
    stations_properties = read_stations_properties_map(ctx)
    pass
