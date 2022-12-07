BASE_URL = "https://review.px4.io/download"
DEFAULT_DOWNLOAD = 'dataDownloaded'
RADIUS_OF_EARTH = 6371000

# all AUTO-modes
# modified from https://github.com/PX4/flight_review
UAV_STATUS = {
    # 3: 'Mission',
    # 4: 'Loiter',
    # 5: 'Return to Land',
    # 6: 'RC Recovery',
    # 7: 'Return to groundstation',
    # 8: 'Land (engine fail)',
    # 9: 'Land (GPS fail)',
    # 12: 'Descend',
    # 13: 'Terminate',
    # 17: 'Takeoff',
    # 18: 'Land',
    # 19: 'Follow Target',
    # 20: 'Precision Land',
    # 21: 'Orbit',
    'Mission': 3,
    'Loiter': 4,
    'Return to Land': 5,
    'RC Recovery': 6,
    'Return to groundstation': 7,
    'Land (engine fail)': 8,
}
