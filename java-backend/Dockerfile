FROM eclipse-temurin:21-jdk

# Gradle itself isn't installed here on purpose - once you run
# `gradle init` (or `gradle wrapper`) on your own machine, the project
# gets its own gradlew script that downloads the right Gradle version
# automatically. That's the standard, portable way to do it, so the
# image doesn't need to hardcode a Gradle version at all.

WORKDIR /app

CMD ["sleep", "infinity"]
