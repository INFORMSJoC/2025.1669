# Input instances

Instances 1-15 use a one-day recovery window; instances 16-30 use two days.
Flight schedules and historical delays come from BTS Airline On-Time Performance
records; aircraft information comes from the FAA Aircraft Registration database.
Maintenance events and remaining flying hours were assigned by the authors.

Sources cited in the manuscript:

- [BTS (2026), Airline Service Quality Performance 234](https://www.bts.gov/browse-statistical-products-and-data/bts-publications/airline-service-quality-performance-234-time)
- [FAA (2026), Aircraft Registration](https://registry.faa.gov/AircraftInquiry/Search/NNumberInquiry)

Required files: `config.csv` (recovery window), `flights.csv`, `aircraft.csv`,
`rotations.csv`. Optional disruptions: `alt_flights.csv`, `alt_aircraft.csv`,
`alt_airports.csv`. `position.csv` is used by the `Specified` position rule.
These files use space-separated fields. `config.json` controls solver settings.
