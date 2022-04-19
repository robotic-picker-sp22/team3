

export function round(num: number, decimalPlaces: number) {
    return parseFloat(num.toFixed(decimalPlaces))
}

export function deg2rad(deg: number) {
    return deg * Math.PI / 180
}

export function rad2deg(rad: number) {
    return rad * 180 / Math.PI
}