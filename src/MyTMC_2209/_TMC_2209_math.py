def rps_to_vactual(rps, steps_per_rev, fclk = 12000000):
    """converts rps -> vactual

    Args:
        rps (float): revolutions per second
        steps_per_rev (int): steps per revolution
        fclk (int): clock speed of the tmc (Default value = 12000000)

    Returns:
        vactual (int): value for vactual
    """
    return int(round(rps / (fclk / 16777216) * steps_per_rev))