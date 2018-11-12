#!groovy

def get_stages(ubuntu_version) {
  return {
    node('docker') {
      step([$class: 'StashNotifier'])

      try {
        checkout scm

        docker.build("libfranka-ci-worker:${ubuntu_version}",
                     "-f .ci/Dockerfile.${ubuntu_version} .ci")
              .inside('--cap-add SYS_PTRACE') {
          stage("${ubuntu_version}: Build (Debug)") {
            sh '.ci/debug.sh'
            junit 'build-debug/test_results/*.xml'
          }

          stage("${ubuntu_version}: Build (Release)") {
            sh '.ci/release.sh'
            if (ubuntu_version == "xenial") {
              // Can't use dir() for this shell script due to JENKINS-33510
              sh "cd ${env.WORKSPACE}/build-release/doc && tar cfz ../libfranka-docs.tar.gz html"
              dir('build-release') {
                archive '*.deb, *.tar.gz'
                publishHTML([allowMissing: false,
                             alwaysLinkToLastBuild: false,
                             keepAll: true,
                             reportDir: 'doc/html',
                             reportFiles: 'index.html',
                             reportName: "API Documentation"])
              }
            }
          }

          stage("${ubuntu_version}: Build (Coverage)") {
            sh '.ci/coverage.sh'
            publishHTML([allowMissing: false,
                         alwaysLinkToLastBuild: false,
                         keepAll: true,
                         reportDir: 'build-coverage/coverage',
                         reportFiles: 'index.html',
                         reportName: "Code Coverage (${ubuntu_version})"])
          }

          stage("${ubuntu_version}: Lint") {
            sh '.ci/lint.sh'
          }
        }
        currentBuild.result = 'SUCCESS'
      } catch (e) {
        currentBuild.result = 'FAILED'
      } finally {
        step([$class: 'StashNotifier'])
      }
    }
  }
}

parallel(
  'xenial': get_stages('xenial'),
  'bionic': get_stages('bionic'),
)
