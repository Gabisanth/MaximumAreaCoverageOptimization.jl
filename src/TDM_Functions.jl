# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization

module TDM_Functions
include("Base_Functions.jl")
using Plots
Plots.default(show = true)
using Random


export show_circles
export make_MADS
export allocate_random_circles
#export allocate_even_circles
export Domains_Problem

mutable struct Circle
    x::Float64
    y::Float64
    R::Float64
    Points::Vector{Int}
    Circles::Vector{Int}
end


function show_epoch(circles, cir_domain)
    palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
    for i in eachindex(circles)
        this_color = palette[mod1(i,length(palette))]
        this_cir = circles[i]
        x,y = Base_Functions.draw(this_cir, 0.0, 2*π)
        plot!(x,y, aspect_ratio=1, color = this_color, 
            label = "UAV $i", 
            linewidth = 1.2,
        )
    end
    
    if cir_domain !== nothing
        # Show the shape of domain
        domain_x, domain_y = Base_Functions.draw(cir_domain, 0.0, 2π)
        plot!(domain_x, domain_y,  aspect_ratio=1, color="red",label = "Domain",
        linealpha=1.0, fillalpha=0.5,
        # xlims=(-30,30), ylims=(-30,30),
        linewidth = 5,
        )
        
    end
end


function show_coverage(circles, cir_domain, rnd, M, N)
    palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
    this_cir_group = circles[1+(rnd-1)*N: rnd*N]

    if cir_domain !== nothing 
        # Show the shape of domain
        domain_x, domain_y = Base_Functions.draw(cir_domain, 0.0, 2π)
        lb = ""
        if rnd == M+1
            lb = "Domain"
        end

        # set gradually-changing color
        start_alpha = 0.1
        end_alpha = 1.0
        alphas = range(start_alpha, stop= end_alpha, length=M+1)

        plot!(domain_x, domain_y, aspect_ratio=1, 
            color="red",label = lb,
            linealpha=alphas[rnd], fillalpha=0.5,
            linewidth = 5,
        )
    end

    for i in eachindex(this_cir_group)
        this_color = palette[mod1(i,length(palette))]
        this_cir = this_cir_group[i]
        x, y = Base_Functions.draw(this_cir, 0.0, 2*π)
        if rnd == 1
            plot!(x,y, aspect_ratio=1,
            linewidth = 1.2, 
            color = this_color, 
            label = "UAV $i")
        else
            plot!(x,y, aspect_ratio=1,
            linewidth = 1.2,  
            color = this_color, 
            label =:none)
        end
    end
    
end


function make_MADS(circles)::Vector{Float64}  #makes the array of circle coordinates and radii.
    x_ = []
    y_ = []
    r_ = []
    for i in eachindex(circles)
        this_cir = circles[i]
        push!(x_, this_cir.x)
        push!(y_, this_cir.y)
        push!(r_, this_cir.R)
    end
    return [x_; y_; r_]
end


function allocate_random_circles(cir_domain, N, r_max)
    this_domain = cir_domain.Domain_History[end]
    # Generate circles
    x_arr = Vector{Float64}(undef,0)
    y_arr = Vector{Float64}(undef,0)
    r_arr = Vector{Float64}(undef,0)

    for i in 1:N
        # NOTE: A * rand() -> return float values in [0, A]

        ref_dis = min(0.5* this_domain.R *rand(), r_max)
        ref_angle = 2*π*rand()
        
        x = ref_dis * cos(ref_angle)
        y = ref_dis * sin(ref_angle)
        r = 1 + 10 * rand()
        
        push!(x_arr,x)
        push!(y_arr,y)
        push!(r_arr,r)
    end

    return [x_arr;y_arr;r_arr]

end





mutable struct Domains_Problem
    Domain_History :: Vector{Circle}
    Radius :: Float64
    Expand_rate :: Float64

    domain_expand
    domain_area

    function Domains_Problem(Domain_History, Radius::Float64, Expand_rate::Float64)
        this = new()

        this.Domain_History = Domain_History

        this.Radius = Radius
        push!(this.Domain_History, Circle(0,0, this.Radius,[],[]))

        # this.Total_epochs = Total_epochs
        this.Expand_rate = Expand_rate

        this.domain_expand = function ()
            this.Radius = this.Radius + this.Expand_rate
            push!(this.Domain_History, Circle(0,0, this.Radius,[],[]))
        end

        this.domain_area = function (cir::Circle)
            return pi*cir.R^2
        end

        return this
    end

end


end