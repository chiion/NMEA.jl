module NMEA

export NMEAData, parse_msg!, GGA,
       RMC, GSA, GSV, SVData,
       GBS, VTG, GLL, ZDA,
       DTM, parse

@static if VERSION < v"0.7.0-DEV.2005"
    occursin = ismatch
end


"""
    parse(line::AbstractString)

Parses an NMEA sentence, returning a corresponding type.
"""
function parse(line::AbstractString)

    message, checksum  = split(line, '*')
    checksum = Base.parse(UInt8, checksum, base=16)

    if checksum!=xor(Vector{UInt8}(split(message,"\$")[2])...)
        @warn "Message checksum mismatch"
        # print(message)
        # print(Char(checksum))
        # print(Char(xor(Vector{UInt8}(split(message,"\$")[2])...)))
    end

    items = split(message, ',')

    system = get_system(items[1])

    if (occursin(r"DTM$", items[1]))
        return parse_DTM(items, system)
    elseif (occursin(r"GBS$", items[1]))
        return parse_GBS(items, system)
    elseif (occursin(r"GGA$", items[1]))
        return parse_GGA(items, system)
    elseif (occursin(r"GLL$", items[1]))
        return parse_GLL(items, system)
    elseif (occursin(r"GSA$", items[1]))
        return parse_GSA(items, system)
    elseif (occursin(r"GSV$", items[1]))
        return parse_GSV(items, system)
    elseif (occursin(r"RMC$", items[1]))
        return parse_RMC(items, system)
    elseif (occursin(r"VTG$", items[1]))
        return parse_VTG(items, system)
    elseif (occursin(r"ZDA$", items[1]))
        return parse_ZDA(items, system)
    elseif (occursin(r"PASHR$", items[1]))
        return parse_PASHR(items, system)
    elseif (occursin(r"TXT$", items[1]))
        return nothing
    end

    throw(ArgumentError("NMEA string not supported"))

end

#----------
# type for GGA message - Global Positioning System Fix Data
#----------
mutable struct GGA
    system::String # GPS, GLONASS, GALILEO, or Combined
    time::Union{Int, Nothing}
    latitude::Union{Float64, Nothing} # decimal degrees
    longitude::Union{Float64, Nothing} # decimal degrees
    fix_quality::String
    num_sats::Int
    HDOP::Union{Float64, Nothing}
    altitude::Union{Float64, Nothing} # MSL in meters
    geoidal_seperation::Union{Float64, Nothing} # meters
    age_of_differential::Union{Float64, Nothing} # seconds since last SC104
    diff_reference_id::Union{Int, Nothing} # differential reference station id
    valid::Bool

    function GGA(sys::AbstractString)
        system              = sys
        time                = nothing
        latitude            = nothing
        longitude           = nothing
        fix_quality         = "UNKNOWN"
        num_sats            = 0
        HDOP                = nothing
        altitude            = nothing
        geoidal_seperation  = nothing
        age_of_differential = nothing
        diff_reference_id   = nothing
        valid               = false
        new(system, time, latitude,
            longitude, fix_quality, num_sats,
            HDOP, altitude, geoidal_seperation,
            age_of_differential, diff_reference_id,
            valid)
    end # constructor GGA

end # type GGA

#----------
# GSA message type - GNSS DOP and Active Satellites
#----------
mutable struct GSA
    system::String
    mode::Char
    current_mode::Int
    sat_ids::AbstractVector{Int}
    PDOP::Union{Float64, Nothing}
    HDOP::Union{Float64, Nothing}
    VDOP::Union{Float64, Nothing}
    valid::Bool

    function GSA(sys::AbstractString)
        system       = sys
        mode         = 'M'
        current_mode = 0
        sat_ids      = Int[]
        PDOP         = nothing
        HDOP         = nothing
        VDOP         = nothing
        valid        = false
        new(system, mode, current_mode,
            sat_ids, PDOP, HDOP,
            VDOP, valid)
    end # constructor GSA
end # type GSA

#----------
# ZDA message type - Time and Date
#----------
mutable struct ZDA
    system::String
    time::Union{Int, Nothing}
    day::Union{Int, Nothing}
    month::Union{Int, Nothing}
    year::Union{Int, Nothing}
    zone_hrs::Union{Int, Nothing}
    zone_mins::Union{Int, Nothing}
    valid::Bool

    function ZDA(sys::AbstractString)
        system    = sys
        time      = nothing
        day       = nothing
        month     = nothing
        year      = nothing
        zone_hrs  = nothing
        zone_mins = nothing
        valid     = false
        new(system, time, day,
            month, year, zone_hrs,
            zone_mins, valid)
    end # constructor ZDA

end # type ZDA

#----------
# GBS message type - RAIM GNSS Satellite Fault Detection
#----------
mutable struct GBS
    system::String
    time::Union{Int, Nothing}
    lat_error::Union{Float64, Nothing}
    long_error::Union{Float64, Nothing}
    alt_error::Union{Float64, Nothing}
    failed_PRN::Union{Int, Nothing}
    prob_of_missed::Union{Float64, Nothing}
    excluded_meas_err::Union{Float64, Nothing}
    standard_deviation::Union{Float64, Nothing}
    valid::Bool

    function GBS(sys::AbstractString)
        system             = sys
        time               = nothing
        lat_error          = nothing
        long_error         = nothing
        alt_error          = nothing
        failed_PRN         = 0
        prob_of_missed     = nothing
        excluded_meas_err  = nothing
        standard_deviation = nothing
        valid              = false
        new(system, time, lat_error,
            long_error, alt_error, failed_PRN,
            prob_of_missed, excluded_meas_err, standard_deviation,
            valid)
    end # constructor GBS
end # type GBS

#----------
# GLL message type - Geographic Position –
# Latitude/Longitude
#----------
mutable struct GLL
    system::String
    latitude::Union{Float64, Nothing}
    longitude::Union{Float64, Nothing}
    time::Union{Int, Nothing}
    status::Bool
    mode::Char
    valid::Bool

    function GLL(sys::AbstractString)
        system    = sys
        latitude  = nothing
        longitude = nothing
        time      = nothing
        status    = false
        mode      = 'N'
        valid     = false
        new(system, latitude, longitude,
            time, status, mode,
            valid)
    end # constructor GLL
end # type GLL

#----------
# type to store SV data fields in GSV
#----------
mutable struct SVData
    PRN::Union{Int, Nothing}
    elevation::Union{Int, Nothing}
    azimuth::Union{Int, Nothing}
    SNR::Union{Int, Nothing}

    function SVData()
        PRN       = nothing
        elevation = nothing
        azimuth   = nothing
        SNR       = nothing
        new(PRN, elevation, azimuth,
            SNR)
    end # constructor SVData
end # type SVData

#-----------
# type for GSV messages - GNSS Satellites In View
#-----------
mutable struct GSV
    system::String
    msg_total::Int
    msg_num::Int
    sat_total::Int
    SV_data::AbstractVector{SVData}
    valid::Bool

    function GSV(sys::AbstractString)
        system    = sys
        msg_total = 0
        msg_num   = 0
        sat_total = 0
        SV_data   = SVData[]
        valid     = false
        new(system, msg_total, msg_num,
            sat_total, SV_data, valid)
    end # constructor GSV
end # type GSV

#----------
# RMC message type - Recommended Minimum Specific GNSS Data
#----------
mutable struct RMC
    system::String
    time::Union{Int, Nothing}
    status::Bool
    latitude::Union{Float64, Nothing}
    longitude::Union{Float64, Nothing}
    sog::Union{Float64, Nothing}
    cog::Union{Float64, Nothing}
    date::String
    magvar::Union{Float64, Nothing}
    mode::Char
    valid::Bool

    function RMC(sys::AbstractString)
        system    = sys
        time      = nothing
        status    = false
        latitude  = nothing
        longitude = nothing
        sog       = nothing
        cog       = nothing
        date      = ""
        magvar    = nothing
        mode      = 'N'
        valid     = false
        new(system, time, status,
            latitude, longitude, sog,
            cog, date, magvar,
            mode, valid)
    end # constructor RMC
end # type RMC

#----------
# VTG message type - Course over Ground & Ground Speed
#----------
mutable struct VTG
    CoG_true::Union{Float64, Nothing}
    CoG_mag::Union{Float64, Nothing}
    SoG_knots::Union{Float64, Nothing}
    SoG_kmhr::Union{Float64, Nothing}
    mode::Char
    valid::Bool

    function VTG(sys::AbstractString)
        CoG_true  = nothing
        CoG_mag   = nothing
        SoG_knots = nothing
        SoG_kmhr  = nothing
        mode      = 'N'
        valid     = false
        new(CoG_true, CoG_mag, SoG_knots,
            SoG_kmhr, mode, valid)
    end # constructor VTG

end # type VTG

#----------
# DTM message type - Datum
#----------
mutable struct DTM
    system::String
    local_datum_code::String
    local_datum_subcode::String
    lat_offset::Union{Float64, Nothing}
    long_offset::Union{Float64, Nothing}
    alt_offset::Union{Float64, Nothing}
    ref_datum::String
    valid::Bool

    function DTM(sys::AbstractString)
        system              = sys
        local_datum_code    = ""
        local_datum_subcode = ""
        lat_offset          = nothing
        long_offset         = nothing
        alt_offset          = nothing
        ref_datum           = ""
        valid               = false

        new(system, local_datum_code, local_datum_subcode,
            lat_offset, long_offset, alt_offset,
            ref_datum, valid)
    end # constructor DTM
end # type DTM


mutable struct PASHR
    system::String
    time::Union{Int, Nothing}
    heading::Union{Float64, Nothing}
    heading_type::String
    roll::Union{Float64, Nothing}
    pitch::Union{Float64, Nothing}
    heave::Union{Float64, Nothing}
    roll_accuracy::Union{Float64, Nothing}
    pitch_accuracy::Union{Float64, Nothing}
    heading_accuracy::Union{Float64, Nothing}
    aiding_code::Int
    ins_code::Int
    valid::Bool

    function PASHR(sys::AbstractString)
        system  = sys
        time = nothing
        heading = nothing
        heading_type = "T"
        roll = nothing
        pitch = nothing
        heave = nothing
        roll_accuracy = nothing
        pitch_accuracy = nothing
        heading_accuracy = nothing
        aiding_code = 0
        ins_code = 0
        valid    = false
        new(system, heading, heading_type, roll, pitch, heave, roll_accuracy,
         pitch_accuracy, heading_accuracy, aiding_code, ins_code, valid)
    end

end # type PASHR

#----------
# module handler
#----------
mutable struct NMEAData
    last_GGA::GGA
    last_RMC::RMC
    last_GSA::GSA
    last_GSV::GSV
    last_GBS::GBS
    last_VTG::VTG
    last_GLL::GLL
    last_ZDA::ZDA
    last_DTM::DTM
    last_PASHR::PASHR

    function NMEAData()
        last_GGA = GGA("UNKNOWN")
        last_RMC = RMC("UNKNOWN")
        last_GSA = GSA("UNKNOWN")
        last_GSV = GSV("UNKNOWN")
        last_GBS = GBS("UNKNOWN")
        last_VTG = VTG("UNKNOWN")
        last_GLL = GLL("UNKNOWN")
        last_ZDA = ZDA("UNKNOWN")
        last_DTM = DTM("UNKNOWN")
        last_PASHR = PASHR("UNKNOWN")
        new(last_GGA, last_RMC, last_GSA,
            last_GSV, last_GBS, last_VTG,
            last_GLL, last_ZDA, last_DTM, last_PASHR)
    end # constructor NMEAData
end # type NMEAData

#----------
# open serial port and start reading NMEAData messages
#----------
function parse_msg!(s::NMEAData, line::AbstractString)
    message = split(line, '*')[1]
    items = split(message, ',')

    # get system name
    system = get_system(items[1])

    mtype = ""
    if (occursin(r"DTM$", items[1]))
        s.last_DTM = parse_DTM(items, system)
        mtype = "DTM"

    elseif (occursin(r"GBS$", items[1]))
        s.last_GBS = parse_GBS(items, system)
        mtype = "GBS"

    elseif (occursin(r"GGA$", items[1]))
        s.last_GGA = parse_GGA(items, system)
        mtype = "GGA"

    elseif (occursin(r"GLL$", items[1]))
        s.last_GLL = parse_GLL(items, system)
        mtype = "GLL"

    elseif (occursin(r"GNS$", items[1]))
        mtype = "GNS"

    elseif (occursin(r"GSA$", items[1]))
        s.last_GSA = parse_GSA(items, system)
        mtype = "GSA"

    elseif (occursin(r"GSV$", items[1]))
        s.last_GSV = parse_GSV(items, system)
        mtype = "GSV"

    elseif (occursin(r"RMC$", items[1]))
        s.last_RMC = parse_RMC(items, system)
        mtype = "RMC"

    elseif (occursin(r"VTG$", items[1]))
        s.last_VTG = parse_VTG(items, system)
        mtype = "VTG"

    elseif (occursin(r"ZDA$", items[1]))
        s.last_ZDA = parse_ZDA(items, system)
        mtype = "ZDA"

    elseif (occursin(r"PASHR$", items[1]))
        s.last_PASHR = parse_PASHR(items, system)
        mtype = "PASHR"

    else
        mtype = "PROPRIETARY"
    end
    mtype
end # function parse_msg!

#----------
# determines system from message type string
#----------
function get_system(mtype::SubString)
    system = ""

    # GPS
    if (occursin(r"^\$GP", mtype))
        system = "GPS"

    # GLONASS
    elseif (occursin(r"^\$GL", mtype))
        system = "GLONASS"

    # GALILEO
    elseif (occursin(r"^\$GA", mtype))
        system = "GALILEO"

    # Combined
    elseif (occursin(r"^\$GN", mtype))
        system = "COMBINED"

    # Proprietary (non-NMEA standard) message
    else
        system = "UNKNOWN"
    end

    system
end # function get_system

#----------
# parses GGA messages and returns populated GGA type
#----------
function parse_GGA(items::Array{T}, system::AbstractString) where T <: SubString
    GGA_data = GGA(system)
    GGA_data.time = _hms_to_secs(items[2])
    GGA_data.latitude = _dms_to_dd(items[3], items[4])
    GGA_data.longitude = _dms_to_dd(items[5], items[6])

    fix_flag = tryparse(Int, items[7])
    if (fix_flag == 0)
        GGA_data.fix_quality = "INVALID"
    elseif (fix_flag == 1)
        GGA_data.fix_quality = "GPS (SPS)"
    elseif (fix_flag == 2)
        GGA_data.fix_quality = "DGPS"
    elseif (fix_flag == 3)
        GGA_data.fix_quality = "PPS"
    elseif (fix_flag == 4)
        GGA_data.fix_quality = "REAL TIME KINEMATIC"
    elseif (fix_flag == 5)
        GGA_data.fix_quality = "FLOAT RTK"
    elseif (fix_flag == 6)
        GGA_data.fix_quality = "DEAD RECKONING"
    elseif (fix_flag == 7)
        GGA_data.fix_quality = "MANUAL INPUT"
    elseif (fix_flag == 8)
        GGA_data.fix_quality = "SIMULATION"
    else
        GGA_data.fix_quality = "UNKNOWN"
    end

    GGA_data.num_sats            = tryparse(Int, items[8])
    GGA_data.HDOP                = tryparse(Float64, items[9])
    GGA_data.altitude            = tryparse(Float64, items[10])
    GGA_data.geoidal_seperation  = tryparse(Float64, items[12])
    GGA_data.age_of_differential = tryparse(Float64, items[14])
    GGA_data.diff_reference_id   = tryparse(Int, items[15])
    GGA_data.valid               = true
    GGA_data
end # function parse_GGA

#----------
# parse GSA messages
#----------
function parse_GSA(items::Array{T}, system::AbstractString)  where T <: SubString
    GSA_data = GSA(system)
    GSA_data.mode = items[2][1]
    GSA_data.current_mode = tryparse(Int, items[3])

    for i = 4:length(items) - 3
        if (items[i] == "")
            break
        end
        push!(GSA_data.sat_ids, tryparse(Int, items[i]))
    end

    GSA_data.PDOP  = tryparse(Float64, items[end - 2])
    GSA_data.HDOP  = tryparse(Float64, items[end - 1])
    GSA_data.VDOP  = tryparse(Float64, items[end])
    GSA_data.valid = true
    GSA_data
end # function parse_GSA

#----------
# parse ZDA message
#----------
function parse_ZDA(items::Array{T}, system::AbstractString) where T <: SubString
    ZDA_data = ZDA(system)
    ZDA_data.time      = _hms_to_secs(items[2])
    ZDA_data.day       = tryparse(Int, items[3])
    ZDA_data.month     = tryparse(Int, items[4])
    ZDA_data.year      = tryparse(Int, items[5])
    ZDA_data.zone_hrs  = tryparse(Int, items[6])
    ZDA_data.zone_mins = tryparse(Int, items[7])
    ZDA_data.valid     = true
    ZDA_data
end # function parse_ZDA

#----------
# parse GBS messages
#----------
function parse_GBS(items::Array{T}, system::AbstractString) where T <: SubString
    GBS_data                    = GBS(system)
    GBS_data.time               = _hms_to_secs(items[2])
    GBS_data.lat_error          = tryparse(Float64, items[3])
    GBS_data.long_error         = tryparse(Float64, items[4])
    GBS_data.alt_error          = tryparse(Float64, items[5])
    GBS_data.failed_PRN         = tryparse(Int, items[6])
    GBS_data.prob_of_missed     = tryparse(Float64, items[7])
    GBS_data.excluded_meas_err  = tryparse(Float64, items[8])
    GBS_data.standard_deviation = tryparse(Float64, items[9])
    GBS_data.valid              = true
    GBS_data
end # function parse_GBS

#----------
# parse GLL message
#----------
function parse_GLL(items::Array{T}, system::AbstractString) where T <: SubString
    GLL_data           = GLL(system)
    GLL_data.latitude  = _dms_to_dd(items[2], items[3])
    GLL_data.longitude = _dms_to_dd(items[4], items[5])
    GLL_data.time      = _hms_to_secs(items[6])

    if (items[7] == "A")
        GLL_data.status = true
    else
        GLL_data.status = false
    end

    if (items[8] != "")
        GLL_data.mode = items[8][1]
    end

    GLL_data.valid = true
    GLL_data
end # function parse_GLL

#----------
# parse GSV messages
#----------
function parse_GSV(items::Array{T}, system::AbstractString) where T <: SubString
    GSV_data           = GSV(system)
    GSV_data.msg_total = tryparse(Int, items[2])
    GSV_data.msg_num   = tryparse(Int, items[3])
    GSV_data.sat_total = tryparse(Int, items[4])

    i = 5
    while i < length(items)
        svd           = SVData()
        svd.PRN       = tryparse(Int, items[i])
        svd.elevation = tryparse(Int, items[i + 1])
        svd.azimuth   = tryparse(Int, items[i + 2])
        svd.SNR       = tryparse(Int, items[i + 3])
        push!(GSV_data.SV_data, svd)
        i += 4
    end

    GSV_data.valid = true
    GSV_data
end # function parse_GSV

#----------
# parse RMC messages
#----------
function parse_RMC(items::Array{T}, system::AbstractString) where T <: SubString
    RMC_data = RMC(system)
    RMC_data.time = _hms_to_secs(items[2])

    if (items[3] == "A")
        RMC_data.status = true
    else
        RMC_data.status = false
    end

    RMC_data.latitude  = _dms_to_dd(items[4], items[5])
    RMC_data.longitude = _dms_to_dd(items[6], items[7])
    RMC_data.sog       = tryparse(Float64, items[8])
    RMC_data.cog       = tryparse(Float64, items[9])

    if length(items[10]) == 6
        RMC_data.date      = string(items[10][3:4], '/', items[10][1:2], '/', items[10][5:6])
    end

    if (items[12] == "W" || items[12] == "S")
        RMC_data.magvar = tryparse(Float64, items[11]) * -1
    else
        RMC_data.magvar = tryparse(Float64, items[11])
    end
    RMC_data.mode = items[13][1]
    RMC_data.valid = true
    RMC_data
end # function parse_RMC

#----------
# parses VTG messages
#----------
function parse_VTG(items::Array{T}, system::AbstractString) where T <: SubString
    VTG_data = VTG(system)
    VTG_data.CoG_true  = tryparse(Float64, items[2])
    VTG_data.CoG_mag   = tryparse(Float64, items[4])
    VTG_data.SoG_knots = tryparse(Float64, items[6])
    VTG_data.SoG_kmhr  = tryparse(Float64, items[8])
    VTG_data.mode      = items[10][1]
    VTG_data.valid     = true
    VTG_data
end # function parse_VTG

#----------
# parse DTM messages
#----------
function parse_DTM(items::Array{T}, system::AbstractString) where T <: SubString
    DTM_data = DTM(system)
    DTM_data.local_datum_code = items[2]
    DTM_data.local_datum_subcode = items[3]
    lat_offset = tryparse(Float64, items[4])
    if (items[5] == "S")
        DTM_data.lat_offset = lat_offset * -1
    else
        DTM_data.lat_offset = lat_offset
    end

    long_offset = tryparse(Float64, items[6])
    if (items[7] == "W")
        DTM_data.long_offset = long_offset * -1
    else
        DTM_data.long_offset = long_offset
    end

    DTM_data.alt_offset = tryparse(Float64, items[8])
    DTM_data.ref_datum  = items[9]
    DTM_data.valid      = true
    DTM_data
end # function parse_DTM

function parse_PASHR(items::Array{T}, system::AbstractString) where T<:SubString
    PASHR_data = PASHR(system)
    PASHR_data.time = _hms_to_secs(items[2])
    PASHR_data.heading = tryparse(Float64, items[3])
    if (items[4]=="T")
        PASHR_data.heading_type = "True"
    else
        PASHR_data.heading_type = ""
    end
    PASHR_data.roll = tryparse(Float64, items[5])
    PASHR_data.pitch = tryparse(Float64, items[6])
    PASHR_data.heave = tryparse(Float64, items[7])
    PASHR_data.roll_accuracy = tryparse(Float64, items[8])
    PASHR_data.pitch_accuracy = tryparse(Float64, items[9])
    PASHR_data.heading_accuracy = tryparse(Float64, items[10])
    PASHR_data.aiding_code = items[11]
    if length(items)>11
        # INS status may be missing from certain systems
        PASHR_data.ins_code = items[12]
    else
        PASHR_data.ins_code = 0
    end
    PASHR_data.valid = true
    PASHR_data
end

#----------
# convert degrees minutes seconds to decimal degrees
#----------
function _dms_to_dd(dms::SubString, hemi::SubString)
    if length(dms) == 0
        return nothing
    end

    if (dms[1:1] == "0")
        dms = dms[2:end]
    end

    decimalindex = findfirst('.', dms)
    degrees = Base.parse(Float64, dms[1:decimalindex-3])
    minutes = Base.parse(Float64, dms[decimalindex-2:end])
    dec_degrees = degrees + (minutes / 60)

    if (hemi == "S" || hemi == "W")
        dec_degrees *= -1
    end

    return dec_degrees
end # function _dms_to_dd

#----------
# hhmmss.s-s to time of day in seconds
#----------
function _hms_to_secs(hms::SubString)
    if length(hms) == 0
        return nothing
    end

    hours   = Base.parse(Float64, hms[1:2])
    minutes = Base.parse(Float64, hms[3:4])
    seconds = Base.parse(Float64, hms[5:end])

    return (hours * 3600) + (minutes * 60) + seconds
end # function _hms_to_secs

end # module NMEA
