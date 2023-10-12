using MaximumAreaCoverageOptimization
using Test

@testset "MaximumAreaCoverageOptimization.jl" begin
    @test MaximumAreaCoverageOptimization.greet_my_package() != "Hello world!"
    @test MaximumAreaCoverageOptimization.greet_my_package() == "Hello!"
end

