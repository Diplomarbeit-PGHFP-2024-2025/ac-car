import datetime
from typing import Tuple

from aca_protocols import PropertyData
from uagents import Context

json_key_stations_properties_map: str = "stations_properties_map"


def _read_stations_properties_map(ctx: Context) -> list[Tuple[str, PropertyData]]:
    ctx.logger.info("[Sort Stations, read_stations_properties_map]: Starting reading")
    serialized_properties: list[Tuple[str, str]] = ctx.storage.get(
        json_key_stations_properties_map
    )

    unserialized_properties: list[Tuple[str, PropertyData]] = []

    for item in serialized_properties:
        unserialized_properties.append((item[0], PropertyData.from_json(item[1])))

    return unserialized_properties


def _save_stations_properties_map(
        ctx: Context,
        serialized: list[Tuple[str, PropertyData]],
        save_name: str = json_key_stations_properties_map,
):
    ctx.logger.info(
        f"[Sort Stations, serialize_and_save_stations_properties_map]: Starting serializing and saving to {save_name}"
    )
    to_save: list[Tuple[str, str]] = []

    for item in serialized:
        to_save.append((item[0], item[1].to_json()))

    ctx.storage.set(save_name, to_save)


def _read_car_properties(ctx: Context) -> PropertyData:
    ctx.logger.info("[Sort Stations, read_car_properties]: Starting reading")

    return PropertyData(
        green_energy=ctx.storage.get("green_energy_weight"),
        cost_per_kwh=ctx.storage.get("cost_pet_kwh_weight"),
        charging_wattage=ctx.storage.get("charging_wattage_weight"),
    )


def initialize_stations_properties_map(ctx: Context):
    ctx.storage.set(json_key_stations_properties_map, [])


def set_PropertyData_of_sender(ctx: Context, sender: str, properties: str):
    if ctx.storage.get("finished_waiting"):
        ctx.logger.info(
            f"[Sort Stations, set_PropertyData_of_sender]: Ignoring properties of {sender}"
        )
        return

    ctx.logger.info(
        f"[Sort Stations, set_PropertyData_of_sender]: Setting properties of {sender} to {properties}"
    )
    stations_properties: list[Tuple[str, str]] = ctx.storage.get(
        json_key_stations_properties_map
    )

    stations_properties.append((sender, properties))

    ctx.storage.set(json_key_stations_properties_map, stations_properties)


def initialize_car_properties(ctx: Context):
    if not ctx.storage.get("green_energy_weight"):
        ctx.storage.set("green_energy_weight", 1)

    if not ctx.storage.get("cost_per_kwh_weight"):
        ctx.storage.set("cost_per_kwh_weight", 5)

    if not ctx.storage.get("charging_wattage_weight"):
        ctx.storage.set("charging_wattage_weight", 1)


def sort_stations(ctx: Context) -> (str, PropertyData, Tuple[int, int]):
    stations_properties = _read_stations_properties_map(ctx)

    if not stations_properties:  # case no station responded
        ctx.logger.warning(
            f"[Sort Stations, sort_stations]: No station responded. Here is nothing to filter: {stations_properties}. So there is no optimal station"
        )

        return "NO STATION", None, None

    ctx.logger.info(
        f"[Sort Stations, sort_stations]: Starting to sort: {stations_properties}"
    )

    car_properties = _read_car_properties(ctx)

    #cost_per_kwh
    stations_price_sorted = sorted(
        stations_properties,
        key=lambda to_sort: to_sort[1].cost_per_kwh
    )

    stations_price_len = len(stations_price_sorted)

    price_weighted_result = [
        (station, 5 - 4 * i / (stations_price_len - 1) * car_properties.cost_per_kwh)
        if stations_price_len > 1 else (station, 5)
        for i, station in enumerate(stations_price_sorted)
    ]

    #charging_wattage
    stations_charging_wattage_sorted = sorted(
        stations_properties,
        key=lambda to_sort: to_sort[1].charging_wattage
    )

    stations_charging_wattage_len = len(stations_charging_wattage_sorted)

    charging_wattage_weighted_result = [
        (station, 5 - 4 * i / (stations_charging_wattage_len - 1) * car_properties.charging_wattage)
        if stations_charging_wattage_len > 1 else (station, 5)
        for i, station in enumerate(stations_charging_wattage_sorted)
    ]

    #green_energy
    stations_green_energy_sorted = sorted(
        stations_properties,
        key=lambda to_sort: to_sort[1].charging_wattage
    )

    stations_green_energy_len = len(stations_green_energy_sorted)

    green_energy_weighted_result = [
        (station, 5 - 4 * i / (stations_green_energy_len - 1) * car_properties.green_energy)
        if stations_green_energy_len > 1 else (station, 5)
        for i, station in enumerate(stations_green_energy_sorted)
    ]