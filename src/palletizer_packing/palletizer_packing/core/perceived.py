"""Merge the perceived height map with the placement record. No ROS imports.

Perception wins where it disagrees with the record, but only on cells where both are locally
unambiguous: the record is flat over the cell's 3 x 3 neighbourhood (not a box edge, where
1 cm cells straddle two heights) and the perceived value is consistent with its observed
neighbours. Unobserved cells (occluded, NaN) keep the record. Every disagreement region is
returned so it can be logged.
"""

import numpy as np
from scipy import ndimage


def merge(record, perceived, tol=0.015):
    """(merged height map, [dict(cells, max_diff, centre (iv, iu), perceived, record)])."""
    rec = np.asarray(record, float)
    per = np.asarray(perceived, float)
    obs = np.isfinite(per)
    flat_rec = (ndimage.maximum_filter(rec, 3, mode='nearest') -
                ndimage.minimum_filter(rec, 3, mode='nearest')) < 1e-6
    p0 = np.where(obs, per, -1.0)
    p1 = np.where(obs, per, 10.0)
    flat_per = obs & ((ndimage.maximum_filter(p0, 3, mode='nearest') -
                       ndimage.minimum_filter(p1, 3, mode='nearest')) < tol)
    diff = np.where(obs, per - rec, 0.0)
    dis = flat_rec & flat_per & (np.abs(diff) > tol)
    merged = np.where(dis, per, rec)
    regions = []
    if dis.any():
        lab, n = ndimage.label(dis)
        for i in range(1, n + 1):
            m = lab == i
            iv, iu = np.nonzero(m)
            k = int(np.argmax(np.abs(diff[m])))
            regions.append({'cells': int(m.sum()), 'max_diff': float(diff[m][k]),
                            'centre': (float(iv.mean()), float(iu.mean())),
                            'perceived': float(per[m][k]), 'record': float(rec[m][k])})
    return merged, regions
