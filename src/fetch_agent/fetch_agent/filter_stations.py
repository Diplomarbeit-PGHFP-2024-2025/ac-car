from aca_protocols import PropertyData
from typing import Tuple
from uagents import Context
import datetime

json_key_stations_properties_map: str = "stations_properties_map"


def _read_stations_properties_map(ctx: Context) -> list[Tuple[str, PropertyData]]:
    ctx.logger.info("[Filter Stations, read_stations_properties_map]: Starting reading")
    serialized_properties: list[Tuple[str, str]] = ctx.storage.get(
        json_key_stations_properties_map
    )

    unserialized_properties: list[Tuple[str, PropertyData]] = []

    for item in serialized_properties:
        unserialized_properties.append((item[0], PropertyData.from_json(item[1])))

    return unserialized_properties


def _save_stations_properties_map(
        ctx: Context, serialized: list[Tuple[str, PropertyData]], save_name: str = json_key_stations_properties_map
):
    ctx.logger.info(
        f"[Filter Stations, serialize_and_save_stations_properties_map]: Starting serializing and saving to {save_name}"
    )
    to_save: list[Tuple[str, str]] = []

    for item in serialized:
        to_save.append((item[0], item[1].to_json()))

    ctx.storage.set(save_name, to_save)


def _read_car_properties(ctx: Context) -> PropertyData:
    ctx.logger.info("[Filter Stations, read_car_properties]: Starting reading")
    return PropertyData(
        open_time_frames=ctx.storage.get("open_time_frames"),
        geo_point=ctx.storage.get("geo_point"),
        cost_per_kwh=ctx.storage.get("cost_per_kwh"),
        charging_wattage=ctx.storage.get("charging_wattage"),
        green_energy=ctx.storage.get("green_energy"),
    )


def initialize_stations_properties_map(ctx: Context):
    ctx.storage.set(json_key_stations_properties_map, [])


def set_PropertyData_of_sender(ctx: Context, sender: str, properties: str):
    if ctx.storage.get("finished_waiting"):
        ctx.logger.info(
            f"[Filter Stations, set_PropertyData_of_sender]: Ignoring properties of {sender}"
        )
        return

    ctx.logger.info(
        f"[Filter Stations, set_PropertyData_of_sender]: Setting properties of {sender} to {properties}"
    )
    stations_properties: list[Tuple[str, str]] = ctx.storage.get(
        json_key_stations_properties_map
    )

    stations_properties.append((sender, properties))

    ctx.storage.set(json_key_stations_properties_map, stations_properties)


def initialize_car_properties(ctx: Context):
    ctx.storage.set(
        "open_time_frames",
        [
            (
                (datetime.datetime.now() + datetime.timedelta(hours=1)).timestamp(),
                (datetime.datetime.now() + datetime.timedelta(hours=4)).timestamp(),
            ),
        ],
    )

    ctx.storage.set("geo_point", (20.32, 85.52))
    ctx.storage.set("cost_per_kwh", 0.43)
    ctx.storage.set("charging_wattage", 11)
    ctx.storage.set("green_energy", True)


def filter_stations(ctx: Context) -> (str, PropertyData, Tuple[int, int]):
    stations_properties = _read_stations_properties_map(ctx)

    if not stations_properties:  # case no station responded
        ctx.logger.warning(
            f"[Filter Stations, filter_stations]: No station responded. Here is nothing to filter: {stations_properties}. So there is no optimal station"
        )

        return "NO STATION", None, None

    ctx.logger.info(
        f"[Filter Stations, filter_stations]: Starting to filter: {stations_properties}"
    )

    car_properties = _read_car_properties(ctx)

    sorted_stations: list[Tuple[str, PropertyData]] = sorted(
        stations_properties,
        key=lambda to_sort: sort_by_all(car_properties, to_sort[1]),
    )

    _save_stations_properties_map(ctx, sorted_stations, "sorted_stations")

    optimal_station: (str, PropertyData) = sorted_stations[0]

    ctx.logger.info(
        f"[Filter Stations, filter_stations]: Finished filtering. Optimal station: {optimal_station}"
    )

    return (
        optimal_station[0],
        optimal_station[1],
        optimal_station[1].open_time_frames[0],
    )


def sort_by_all(
        car_properties: PropertyData, station_properties: PropertyData
) -> float:
    timeframe_sorting = _sort_by_timeframe(
        car_properties.open_time_frames, station_properties.open_time_frames
    )
    geo_point_sorting = _sort_by_geo_point(
        car_properties.geo_point, station_properties.geo_point
    )
    cost_per_kwh_sorting = _sort_by_cost_per_kwh(
        car_properties.cost_per_kwh, station_properties.cost_per_kwh
    )
    charging_wattage_sorting = _sort_by_charging_wattage(
        car_properties.charging_wattage, station_properties.charging_wattage
    )
    green_energy_sorting = _sort_by_green_energy(
        car_properties.green_energy, station_properties.green_energy
    )

    return (
            timeframe_sorting
            + geo_point_sorting
            + cost_per_kwh_sorting
            + charging_wattage_sorting
            + green_energy_sorting
    ) / 5


def _sort_by_timeframe(
        car_expected_time_frames: list[Tuple[int, int]],
        station_open_time_frames: list[Tuple[int, int]],
) -> int:
    return abs(car_expected_time_frames[0][0] - station_open_time_frames[0][0])


def _sort_by_geo_point(
        car_geo_point: Tuple[float, float], station_geo_point: Tuple[float, float]
) -> float:
    return abs(
        car_geo_point[0]
        - station_geo_point[0]
        + car_geo_point[1]
        - station_geo_point[1]
    )


def _sort_by_cost_per_kwh(car_expected_cost: float, station_cost: float) -> float:
    return abs(car_expected_cost - station_cost)


def _sort_by_charging_wattage(car_expected_wattage: int, station_wattage: int) -> int:
    return abs(car_expected_wattage - station_wattage)


def _sort_by_green_energy(
        car_expected_green_energy: bool, station_green_energy: bool
) -> int:
    if station_green_energy:  # green energy is better in any case
        return 1
    if car_expected_green_energy and not station_green_energy:
        return -1
    return 0
