#!groovy

buildResult = 'NOT_BUILT'

def getStages(ubuntuVersion) {
  return {
    node('docker') {
      try {
        checkout scm

        docker.build("libfranka-ci-worker:${ubuntuVersion}",
                     "-f .ci/Dockerfile.${ubuntuVersion} .ci")
              .inside('--cap-add SYS_PTRACE -e MAKEFLAGS') {
          stage("${ubuntuVersion}: Build (Debug)") {
            sh '.ci/debug.sh'
            junit 'build-debug/test_results/*.xml'
          }

          stage("${ubuntuVersion}: Build (Release)") {
            sh '.ci/release.sh'
            // Can't use dir() for these shell scripts due to JENKINS-33510
            sh "cd ${env.WORKSPACE}/build-release/doc && tar cfz ../libfranka-docs.tar.gz html"
            sh "cd ${env.WORKSPACE}/build-release && rename -e 's/(.tar.gz|.deb)\$/-${ubuntuVersion}\$1/' *.deb *.tar.gz"
            dir('build-release') {
              archive '*.deb, *.tar.gz'
              publishHTML([allowMissing: false,
                           alwaysLinkToLastBuild: false,
                           keepAll: true,
                           reportDir: 'doc/html',
                           reportFiles: 'index.html',
                           reportName: "API Documentation (${ubuntuVersion})"])
            }
          }

          stage("${ubuntuVersion}: Build (Coverage)") {
            sh '.ci/coverage.sh'
            publishHTML([allowMissing: false,
                         alwaysLinkToLastBuild: false,
                         keepAll: true,
                         reportDir: 'build-coverage/coverage',
                         reportFiles: 'index.html',
                         reportName: "Code Coverage (${ubuntuVersion})"])
          }

          stage("${ubuntuVersion}: Lint") {
            sh '.ci/lint.sh'
          }
        }

        if (buildResult != 'FAILED') {
          buildResult = 'SUCCESS'
        }
      } catch (e) {
        println(e)
        buildResult = 'FAILED'
      }
    }
  }
}

node {
  step([$class: 'StashNotifier'])
}

parallel(
  'xenial': getStages('xenial'),
  'bionic': getStages('bionic'),
  'focal': getStages('focal'),
)

node {
  currentBuild.result = buildResult
  step([$class: 'StashNotifier'])
}
