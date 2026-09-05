# Test instance file format

All basic instances must be located in the folder `TestInstances`. Instances
with uncertainty must be located in the folder `RobustInstances`. Each `.txt`
file in the corresponding directory is whitespace-delimited and contains no
section headers. Read the file in the following order:

1. One general-information line.
2. One line for each zone.
3. One line for each task.
4. The zone (tower) travel-time matrix.
5. The task (crew) travel-time matrix.

## 1. General information

The first line has four integer fields:

```text
numberOfCrews numberOfTowers numberOfZones numberOfTasks
```

For example, `8 3 6 20` declares 8 crews, 3 towers, 6 zones, and 20 tasks.

## 2. Zone data

The next `numberOfZones` lines each have five fields:

```text
zoneID positioningTime weight priority numberOfPeople
```

- `zoneID`: integer zone identifier from `1` through `numberOfZones`; ID `0`
  is reserved for the depot.
- `positioningTime`: time required for a tower to position or set up at the
  zone.
- `weight`: numerical weight assigned to the zone.
- `priority`: integer priority assigned to the zone.
- `numberOfPeople`: population associated with the zone.

For example, `1 27 0.1058 5 1327` describes zone 1, with positioning time 27,
weight 0.1058, priority 5, and a population of 1,327.

## 3. Task data

The next `numberOfTasks` lines each have three fields:

```text
taskID repairTime associatedZoneIDs
```

- `taskID`: integer task identifier from `1` through `numberOfTasks`; ID `0`
  is reserved for the depot.
- `repairTime`: time required to complete the task.
- `associatedZoneIDs`: one or more zone IDs. Multiple IDs must be separated by
  commas with no spaces, for example `1,3,5`. Every zone must be associated
  with at least one task.

For example, `1 225 6` defines task 1 with repair time 225 and associates it
with zone 6.

## 4. Zone travel-time matrix

This is a square `(numberOfZones + 1) x (numberOfZones + 1)` matrix. Rows and
columns are ordered `0, 1, 2, ..., numberOfZones`. Entry `(i, j)` is the tower
travel time from location `i` to location `j`. Index `0` denotes the start
depot, and travel back to index `0` is used as the return-to-depot time.

## 5. Task travel-time matrix

This is a square `(numberOfTasks + 1) x (numberOfTasks + 1)` matrix. Rows and
columns are ordered `0, 1, 2, ..., numberOfTasks`. Entry `(i, j)` is the crew
travel time from location `i` to location `j`. Index `0` denotes the start
depot, and travel back to index `0` is used as the return-to-depot time.
